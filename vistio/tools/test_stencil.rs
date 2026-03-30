use std::f32::consts::PI;

fn main() {
    let v0 = (0.0, 0.0);
    let v1 = (2.0, 0.0);
    let wa = (1.0, 1.0);
    let wb = (1.5, -2.0);

    // we want c0, c1, ca, cb such that
    // c0*v0 + c1*v1 + ca*wa + cb*wb = (0,0)
    // c0 + c1 + ca + cb = 0
    // ca = ? cb = ?
    
    // Let's test the hypothesis:
    // ca = cot(v0, v1 at wa) = ? No, wait.
    let vec_a1 = (wa.0 - v0.0, wa.1 - v0.1);
    let vec_a2 = (wa.0 - v1.0, wa.1 - v1.1);
    let cot_wa = (vec_a1.0*vec_a2.0 + vec_a1.1*vec_a2.1) / (vec_a1.0*vec_a2.1 - vec_a1.1*vec_a2.0).abs();
    
    let vec_b1 = (wb.0 - v0.0, wb.1 - v0.1);
    let vec_b2 = (wb.0 - v1.0, wb.1 - v1.1);
    let cot_wb = (vec_b1.0*vec_b2.0 + vec_b1.1*vec_b2.1) / (vec_b1.0*vec_b2.1 - vec_b1.1*vec_b2.0).abs();

    let vec_0a = (v0.0 - wa.0, v0.1 - wa.1);
    let vec_0b = (v0.0 - wb.0, v0.1 - wb.1);
    let vec_01 = (v0.0 - v1.0, v0.1 - v1.1);
    let cot_0_a = (vec_0a.0*vec_01.0 + vec_0a.1*vec_01.1) / (vec_0a.0*vec_01.1 - vec_0a.1*vec_01.0).abs();
    let cot_0_b = (vec_0b.0*vec_01.0 + vec_0b.1*vec_01.1) / (vec_0b.0*vec_01.1 - vec_0b.1*vec_01.0).abs();

    let vec_1a = (v1.0 - wa.0, v1.1 - wa.1);
    let vec_1b = (v1.0 - wb.0, v1.1 - wb.1);
    let vec_10 = (v1.0 - v0.0, v1.1 - v0.1);
    let cot_1_a = (vec_1a.0*vec_10.0 + vec_1a.1*vec_10.1) / (vec_1a.0*vec_10.1 - vec_1a.1*vec_10.0).abs();
    let cot_1_b = (vec_1b.0*vec_10.0 + vec_1b.1*vec_10.1) / (vec_1b.0*vec_10.1 - vec_1b.1*vec_10.0).abs();

    let ca = cot_wa;
    let cb = cot_wb;
    let c0 = -(cot_1_a + cot_1_b); // Wait, cot at v1 ? No, cot at v0 is cot_1_a... wait
    let c1 = -(cot_0_a + cot_0_b); 

    let sum_x = c0*v0.0 + c1*v1.0 + ca*wa.0 + cb*wb.0;
    let sum_y = c0*v0.1 + c1*v1.1 + ca*wa.1 + cb*wb.1;
    let sum_c = c0 + c1 + ca + cb;

    println!("Sum C: {}", sum_c);
    println!("Sum X: {}", sum_x);
    println!("Sum Y: {}", sum_y);
}
