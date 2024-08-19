#![cfg_attr(feature = "no_std", no_std)]
#![cfg_attr(feature = "no_std", no_main)]

#[cfg_attr(feature = "no_std", panic_handler)]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[cfg(not(feature = "no_std"))]
fn main() {}
