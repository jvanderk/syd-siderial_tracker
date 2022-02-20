// utility functions not present in f32 implementation in no_std
pub fn sign(f: f32) -> f32 {
    if f < 0.0 {
        -1.0
    } else {
        1.0
    }
}

pub fn abs(x: f32) -> f32 {
    f32::from_bits(x.to_bits() & 0x7FFF_FFFF)
}
