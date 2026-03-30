// physics/src/utils/mod.rs

pub mod coloring;
pub mod normals;
pub mod profiler;
pub mod simd;

use wasm_bindgen::prelude::*;

pub fn set_panic_hook() {
    console_error_panic_hook::set_once();
}

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console)]
    pub fn log(s: &str);
}

#[macro_export]
macro_rules! console_log {
    ($($t:tt)*) => ($crate::utils::log(&format!($($t)*)))
}