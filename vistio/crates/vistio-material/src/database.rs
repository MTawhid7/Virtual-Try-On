//! Material database with physically-grounded fabric presets.
//!
//! The 5 initial presets correspond to the material library
//! defined in the roadmap (Tier 3), based on published KES data
//! for common fabric types.

use std::collections::HashMap;

use crate::properties::FabricProperties;

/// A named collection of fabric material presets.
///
/// Materials are looked up by name (e.g., "cotton_twill", "silk_charmeuse").
/// Custom materials can be registered at runtime.
#[derive(Debug, Clone)]
pub struct MaterialDatabase {
    materials: HashMap<String, FabricProperties>,
}

impl MaterialDatabase {
    /// Creates a new database with the 5 built-in fabric presets.
    pub fn with_defaults() -> Self {
        let mut db = Self {
            materials: HashMap::new(),
        };

        db.register(FabricProperties::cotton_twill());
        db.register(FabricProperties::silk_charmeuse());
        db.register(FabricProperties::denim_14oz());
        db.register(FabricProperties::jersey_knit());
        db.register(FabricProperties::chiffon());

        db
    }

    /// Creates an empty database.
    pub fn empty() -> Self {
        Self {
            materials: HashMap::new(),
        }
    }

    /// Registers a material. Overwrites if the name already exists.
    pub fn register(&mut self, props: FabricProperties) {
        self.materials.insert(props.name.clone(), props);
    }

    /// Looks up a material by name. Returns `None` if not found.
    pub fn get(&self, name: &str) -> Option<&FabricProperties> {
        self.materials.get(name)
    }

    /// Returns all registered material names.
    pub fn names(&self) -> Vec<&str> {
        self.materials.keys().map(|s| s.as_str()).collect()
    }

    /// Returns the number of registered materials.
    pub fn len(&self) -> usize {
        self.materials.len()
    }

    /// Returns true if the database is empty.
    pub fn is_empty(&self) -> bool {
        self.materials.is_empty()
    }
}

impl Default for MaterialDatabase {
    fn default() -> Self {
        Self::with_defaults()
    }
}

// ─── Built-in Fabric Presets ──────────────────────────────────────────

impl FabricProperties {
    /// Cotton Twill (200gsm) — Medium-weight woven fabric.
    /// Good all-rounder. Holds shape with defined folds.
    pub fn cotton_twill() -> Self {
        FabricProperties {
            name: "cotton_twill".into(),
            density: 200.0,
            stretch_stiffness_warp: 0.90,
            stretch_stiffness_weft: 0.85,
            shear_stiffness: 0.50,
            bending_stiffness_warp: 0.50,
            bending_stiffness_weft: 0.45,
            thickness: 0.0008,
            friction: 0.50,
            damping: 0.02,
        }
    }

    /// Silk Charmeuse (80gsm) — Lightweight, flowing, low structure.
    /// Produces soft drapes with many small folds.
    pub fn silk_charmeuse() -> Self {
        FabricProperties {
            name: "silk_charmeuse".into(),
            density: 80.0,
            stretch_stiffness_warp: 0.70,
            stretch_stiffness_weft: 0.65,
            shear_stiffness: 0.30,
            bending_stiffness_warp: 0.10,
            bending_stiffness_weft: 0.08,
            thickness: 0.0004,
            friction: 0.30,
            damping: 0.01,
        }
    }

    /// Denim 14oz (~400gsm) — Heavy, stiff woven fabric.
    /// Resists deformation, produces sharp creases.
    pub fn denim_14oz() -> Self {
        FabricProperties {
            name: "denim_14oz".into(),
            density: 400.0,
            stretch_stiffness_warp: 0.98,
            stretch_stiffness_weft: 0.92,
            shear_stiffness: 0.70,
            bending_stiffness_warp: 0.85,
            bending_stiffness_weft: 0.75,
            thickness: 0.0014,
            friction: 0.60,
            damping: 0.03,
        }
    }

    /// Jersey Knit (180gsm) — Stretchy knitted fabric.
    /// High stretch, low bending stiffness, flowing drape.
    pub fn jersey_knit() -> Self {
        FabricProperties {
            name: "jersey_knit".into(),
            density: 180.0,
            stretch_stiffness_warp: 0.50,
            stretch_stiffness_weft: 0.60,
            shear_stiffness: 0.25,
            bending_stiffness_warp: 0.20,
            bending_stiffness_weft: 0.25,
            thickness: 0.0006,
            friction: 0.40,
            damping: 0.02,
        }
    }

    /// Chiffon (50gsm) — Ultra-lightweight, sheer, flowing.
    /// Minimal structure, maximum flow.
    pub fn chiffon() -> Self {
        FabricProperties {
            name: "chiffon".into(),
            density: 50.0,
            stretch_stiffness_warp: 0.60,
            stretch_stiffness_weft: 0.55,
            shear_stiffness: 0.20,
            bending_stiffness_warp: 0.05,
            bending_stiffness_weft: 0.04,
            thickness: 0.0003,
            friction: 0.20,
            damping: 0.005,
        }
    }
}
