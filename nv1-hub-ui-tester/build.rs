fn main() {
    let target = std::env::var("CARGO_CFG_TARGET_ARCH").unwrap();

    if target == "arm" {
        println!("cargo:warning=This crate does not support ARM architecture.");
        return;
    }
}
