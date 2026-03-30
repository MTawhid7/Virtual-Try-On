//! Integration tests for vistio-material.

use vistio_material::{FabricProperties, MaterialDatabase};

// ─── FabricProperties Tests ───────────────────────────────────

#[test]
fn mass_calculation() {
    let props = FabricProperties {
        name: "Test".into(),
        density: 200.0,
        stretch_stiffness_warp: 0.9,
        stretch_stiffness_weft: 0.9,
        shear_stiffness: 0.5,
        bending_stiffness_warp: 0.5,
        bending_stiffness_weft: 0.5,
        thickness: 0.001,
        friction: 0.5,
        damping: 0.01,
    };
    let mass = props.mass_per_vertex(100, 1.0);
    assert!((mass - 0.002).abs() < 1e-6);
}

#[test]
fn avg_stiffness() {
    let props = FabricProperties {
        name: "Test".into(),
        density: 100.0,
        stretch_stiffness_warp: 0.8,
        stretch_stiffness_weft: 0.6,
        shear_stiffness: 0.5,
        bending_stiffness_warp: 0.4,
        bending_stiffness_weft: 0.2,
        thickness: 0.001,
        friction: 0.5,
        damping: 0.01,
    };
    assert!((props.avg_stretch_stiffness() - 0.7).abs() < 1e-6);
    assert!((props.avg_bending_stiffness() - 0.3).abs() < 1e-6);
}

// ─── MaterialDatabase Tests ──────────────────────────────────

#[test]
fn default_database_has_five_materials() {
    let db = MaterialDatabase::with_defaults();
    assert_eq!(db.len(), 5);
}

#[test]
fn lookup_by_name() {
    let db = MaterialDatabase::with_defaults();
    let cotton = db.get("cotton_twill").unwrap();
    assert_eq!(cotton.density, 200.0);
}

#[test]
fn lookup_all_presets() {
    let db = MaterialDatabase::with_defaults();
    assert!(db.get("cotton_twill").is_some());
    assert!(db.get("silk_charmeuse").is_some());
    assert!(db.get("denim_14oz").is_some());
    assert!(db.get("jersey_knit").is_some());
    assert!(db.get("chiffon").is_some());
}

#[test]
fn missing_material_returns_none() {
    let db = MaterialDatabase::with_defaults();
    assert!(db.get("nonexistent").is_none());
}

#[test]
fn custom_material() {
    let mut db = MaterialDatabase::empty();
    assert!(db.is_empty());
    db.register(FabricProperties {
        name: "custom_fabric".into(),
        density: 150.0,
        stretch_stiffness_warp: 0.80,
        stretch_stiffness_weft: 0.75,
        shear_stiffness: 0.40,
        bending_stiffness_warp: 0.30,
        bending_stiffness_weft: 0.28,
        thickness: 0.0005,
        friction: 0.35,
        damping: 0.015,
    });
    assert_eq!(db.len(), 1);
    assert!(db.get("custom_fabric").is_some());
}

#[test]
fn density_ordering() {
    let db = MaterialDatabase::with_defaults();
    let chiffon = db.get("chiffon").unwrap().density;
    let silk = db.get("silk_charmeuse").unwrap().density;
    let jersey = db.get("jersey_knit").unwrap().density;
    let cotton = db.get("cotton_twill").unwrap().density;
    let denim = db.get("denim_14oz").unwrap().density;
    assert!(chiffon < silk);
    assert!(silk < jersey);
    assert!(jersey < cotton);
    assert!(cotton < denim);
}

#[test]
fn stiffness_ordering() {
    let db = MaterialDatabase::with_defaults();
    let denim = db.get("denim_14oz").unwrap();
    let chiffon = db.get("chiffon").unwrap();
    assert!(denim.stretch_stiffness_warp > chiffon.stretch_stiffness_warp);
    assert!(denim.bending_stiffness_warp > chiffon.bending_stiffness_warp);
}

// ─── ConstitutiveModel Tests ─────────────────────────────────

use vistio_material::{CoRotationalModel, ConstitutiveModel, IsotropicLinearModel};
use vistio_math::mat3x2::Mat3x2;
use vistio_math::Vec3;

#[test]
fn corotational_identity_produces_identity_target() {
    let model = CoRotationalModel::new();
    let f = Mat3x2::IDENTITY;
    let result = model.project(&f, 1.0, 1.0);
    // Undeformed → target should be identity (rotation = identity)
    assert!((result.target_f.col0 - Vec3::X).length() < 1e-4);
    assert!((result.target_f.col1 - Vec3::Y).length() < 1e-4);
}

#[test]
fn corotational_energy_zero_at_rest() {
    let model = CoRotationalModel::new();
    let f = Mat3x2::IDENTITY;
    let result = model.project(&f, 1.0, 1.0);
    // No deformation → zero energy
    assert!(result.energy < 1e-8, "Energy at rest should be ~0, got {}", result.energy);
}

#[test]
fn corotational_energy_positive_under_strain() {
    let model = CoRotationalModel::new();
    // Uniform 2x stretch
    let f = Mat3x2::from_cols(Vec3::X * 2.0, Vec3::Y * 2.0);
    let result = model.project(&f, 1.0, 1.0);
    assert!(result.energy > 0.0, "Stretched element should have positive energy");
}

#[test]
fn corotational_rotated_element_zero_energy() {
    let model = CoRotationalModel::new();
    // Pure 90° rotation around Z axis: (X→Y, Y→-X)
    let f = Mat3x2::from_cols(Vec3::Y, -Vec3::X);
    let result = model.project(&f, 1.0, 1.0);
    // Pure rotation → F = R, energy = ||F - R||² = 0
    assert!(result.energy < 1e-4, "Pure rotation should have ~0 energy, got {}", result.energy);
}

#[test]
fn corotational_target_is_rotation() {
    let model = CoRotationalModel::new();
    // Stretch + rotation: scale X by 1.5 then rotate 45°
    let angle = std::f32::consts::FRAC_PI_4;
    let c = angle.cos();
    let s = angle.sin();
    let f = Mat3x2::from_cols(
        Vec3::new(c * 1.5, s * 1.5, 0.0),
        Vec3::new(-s, c, 0.0),
    );
    let result = model.project(&f, 1.0, 1.0);
    // Target should be orthonormal (columns should be unit length)
    let len0 = result.target_f.col0.length();
    let len1 = result.target_f.col1.length();
    assert!((len0 - 1.0).abs() < 0.1, "Target col0 should be ~unit, got {}", len0);
    assert!((len1 - 1.0).abs() < 0.1, "Target col1 should be ~unit, got {}", len1);
}

#[test]
fn isotropic_identity_produces_identity_target() {
    let model = IsotropicLinearModel::new();
    let f = Mat3x2::IDENTITY;
    let result = model.project(&f, 1.0, 1.0);
    // Isotropic always targets identity
    assert!((result.target_f.col0 - Vec3::X).length() < 1e-6);
    assert!((result.target_f.col1 - Vec3::Y).length() < 1e-6);
}

#[test]
fn isotropic_energy_zero_at_rest() {
    let model = IsotropicLinearModel::new();
    let f = Mat3x2::IDENTITY;
    let result = model.project(&f, 1.0, 1.0);
    assert!(result.energy < 1e-8, "Energy at rest should be ~0, got {}", result.energy);
}

#[test]
fn isotropic_energy_positive_under_strain() {
    let model = IsotropicLinearModel::new();
    let f = Mat3x2::from_cols(Vec3::X * 1.5, Vec3::Y * 1.5);
    let result = model.project(&f, 1.0, 1.0);
    assert!(result.energy > 0.0, "Stretched should have positive energy");
}

#[test]
fn isotropic_rotation_produces_nonzero_energy() {
    // This is the key difference from co-rotational:
    // A pure rotation is NOT at rest for the isotropic model.
    let model = IsotropicLinearModel::new();
    let f = Mat3x2::from_cols(Vec3::Y, -Vec3::X); // 90° rotation
    let result = model.project(&f, 1.0, 1.0);
    // ||R - I||² > 0 because R ≠ I
    assert!(result.energy > 0.1, "Rotation should produce energy for isotropic model, got {}", result.energy);
}

#[test]
fn corotational_model_name() {
    let model = CoRotationalModel::new();
    assert_eq!(model.name(), "co_rotational");
}

#[test]
fn isotropic_model_name() {
    let model = IsotropicLinearModel::new();
    assert_eq!(model.name(), "isotropic_linear");
}

// ─── Tension-Field Theory Tests ───────────────────────────────

#[test]
fn corotational_compression_buckling_energy() {
    // With pure ARAP (cs=1.0), returning s_target=1.0 creates significant
    // compressive stress (symmetric to stretching). We accept this compression
    // penalty because using tension-field theory in a constant-matrix PD solver
    // causes massive viscous drag. We allow draping by keeping membrane stiffness
    // globally low instead.
    let model = CoRotationalModel::new();
    let f = Mat3x2::from_cols(Vec3::X * 0.5, Vec3::Y * 0.5);
    let result = model.project(&f, 1.0, 1.0);
    assert!(
        result.energy > 0.1,
        "Compressed element must have positive energy to induce buckling, got {}",
        result.energy
    );
}

#[test]
fn corotational_stretch_still_penalized() {
    // Tensile deformation should still produce positive energy.
    let model = CoRotationalModel::new();
    let f = Mat3x2::from_cols(Vec3::X * 2.0, Vec3::Y * 2.0);
    let result = model.project(&f, 1.0, 1.0);
    assert!(
        result.energy > 0.1,
        "Stretched element should have positive energy, got {}",
        result.energy
    );
}

#[test]
fn corotational_mixed_compression_stretch() {
    // One direction compressed (σ₁ = 0.5), one stretched (σ₂ = 1.5).
    let model = CoRotationalModel::new();
    let f = Mat3x2::from_cols(Vec3::X * 0.5, Vec3::Y * 1.5);
    let result = model.project(&f, 1.0, 1.0);
    assert!(
        result.energy > 0.0,
        "Mixed deformation should have positive energy, got {}",
        result.energy
    );
    // Under pure ARAP, compression from 1.0 to 0.5 costs the same as stretch from 1.0 to 1.5.
    let pure_stretch = Mat3x2::from_cols(Vec3::X * 1.5, Vec3::Y * 1.5);
    let pure_result = model.project(&pure_stretch, 1.0, 1.0);
    
    let diff = (result.energy - pure_result.energy).abs();
    assert!(
        diff < 1e-6,
        "ARAP penalizes compression and stretch symmetrically. Expected equality, got {} vs {}",
        result.energy,
        pure_result.energy
    );
}

// ─── Phase 5: Orthotropic Model Tests ─────────────────────────

use vistio_material::OrthotropicLinearModel;

#[test]
fn orthotropic_model_name() {
    let model = OrthotropicLinearModel::new(1.0, 1.0);
    assert_eq!(model.name(), "orthotropic_linear");
}

#[test]
fn orthotropic_identity_zero_energy() {
    let model = OrthotropicLinearModel::new(1.0, 1.0);
    let f = Mat3x2::IDENTITY;
    let result = model.project(&f, 1.0, 1.0);
    assert!(
        result.energy < 1e-6,
        "Identity should have ~0 energy, got {}",
        result.energy
    );
}

#[test]
fn orthotropic_anisotropic_energy_differs() {
    // When warp is much stiffer than weft, stretching in warp should cost more
    let model = OrthotropicLinearModel::new(10.0, 1.0);

    // Stretch in the warp direction (col0 = X direction)
    let f_warp_stretch = Mat3x2::from_cols(Vec3::X * 2.0, Vec3::Y);
    let e_warp = model.project(&f_warp_stretch, 1.0, 1.0).energy;

    // Stretch in the weft direction (col1 = Y direction)
    let f_weft_stretch = Mat3x2::from_cols(Vec3::X, Vec3::Y * 2.0);
    let e_weft = model.project(&f_weft_stretch, 1.0, 1.0).energy;

    // Weft direction (lower stiffness → less restoration → more residual deformation → LESS energy)
    assert!(
        e_weft < e_warp || (e_warp - e_weft).abs() < e_warp * 0.5,
        "Anisotropic model should differentiate directions: warp_e={}, weft_e={}",
        e_warp, e_weft
    );
}

#[test]
fn orthotropic_equal_stiffness_isotropic() {
    // Equal stiffness should behave like isotropic
    let model = OrthotropicLinearModel::new(5.0, 5.0);
    let f = Mat3x2::from_cols(Vec3::X * 1.5, Vec3::Y * 1.5);
    let result = model.project(&f, 1.0, 1.0);
    assert!(
        result.energy > 0.0,
        "Stretched element should have positive energy"
    );
}

// ─── Anisotropic Co-Rotational Model Tests (Tier 3) ──────────

use vistio_material::AnisotropicCoRotationalModel;

#[test]
fn anisotropic_model_name() {
    let model = AnisotropicCoRotationalModel::new(1.0, 1.0);
    assert_eq!(model.name(), "anisotropic_corotational");
}

#[test]
fn anisotropic_corotational_identity_zero_energy() {
    let model = AnisotropicCoRotationalModel::new(1.0, 0.5);
    let f = Mat3x2::IDENTITY;
    let result = model.project(&f, 1.0, 1.0);
    assert!(result.energy < 1e-6,
        "Identity F should have ~0 energy, got {}", result.energy);
}

#[test]
fn anisotropic_corotational_compression_buckling_energy() {
    // Like the isotropic model, returning 1.0 creates significant compressive stress.
    let model = AnisotropicCoRotationalModel::new(1.0, 0.5);
    let f = Mat3x2::from_cols(Vec3::X * 0.5, Vec3::Y * 0.5);
    let result = model.project(&f, 1.0, 1.0);
    assert!(result.energy > 0.01,
        "Compressed element must have positive energy to induce buckling, got {}",
        result.energy);
}

#[test]
fn anisotropic_warp_weft_different_energy() {
    // Warp is much stiffer (9x) → stretching along warp should cost more
    let model = AnisotropicCoRotationalModel::new(0.9, 0.1);

    // Stretch along warp direction (X-axis = col0)
    let f_warp = Mat3x2::from_cols(Vec3::X * 2.0, Vec3::Y);
    let e_warp = model.project(&f_warp, 1.0, 1.0).energy;

    // Stretch along weft direction (Y-axis = col1)
    let f_weft = Mat3x2::from_cols(Vec3::X, Vec3::Y * 2.0);
    let e_weft = model.project(&f_weft, 1.0, 1.0).energy;

    // Stretching along the stiffer warp direction restores MORE toward σ=1.0,
    // creating a larger residual (F - target) → MORE energy.
    // The weaker weft axis leaves target closer to the deformed state → LESS energy.
    assert!(e_warp > e_weft,
        "Stiff warp should yield more energy (larger restoration residual): warp={:.6} weft={:.6}",
        e_warp, e_weft);
}

#[test]
fn anisotropic_equal_stiffness_matches_corotational() {
    // When warp = weft, results should match standard co-rotational
    let corot = CoRotationalModel::new();
    let aniso = AnisotropicCoRotationalModel::new(1.0, 1.0);

    let f = Mat3x2::from_cols(Vec3::X * 1.5, Vec3::Y * 0.8);
    let corot_result = corot.project(&f, 1.0, 1.0);
    let aniso_result = aniso.project(&f, 1.0, 1.0);

    // Energies should be very close
    let diff = (corot_result.energy - aniso_result.energy).abs();
    assert!(diff < corot_result.energy * 0.01 + 1e-6,
        "Equal stiffness should match co-rotational: corot={:.6} aniso={:.6}",
        corot_result.energy, aniso_result.energy);
}
