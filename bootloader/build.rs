use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    generate_memory_layout(out);
}

/// Emits FIRMWARE_ORIGIN and FIRMWARE_SIZE from the two memory.x files
fn generate_memory_layout(out_dir: &PathBuf) {
    println!("cargo:rerun-if-changed=../firmware/memory.x");
    let (origin, length) = flash_region("memory.x");
    let (_, firmware_size) = flash_region("../firmware/memory.x");
    let layout = format!(
        "pub const FIRMWARE_ORIGIN: u32 = {:#x};\npub const FIRMWARE_SIZE: u32 = {:#x};\n",
        origin + length,
        firmware_size,
    );
    std::fs::write(out_dir.join("layout.rs"), layout).expect("write layout.rs");
}

/// ORIGIN and LENGTH of the FLASH region in a linker memory script
fn flash_region(path: &str) -> (u32, u32) {
    let src = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    for line in src.lines() {
        let line = line.split("/*").next().unwrap();
        let Some((name, rest)) = line.split_once(':') else { continue };
        if name.trim() != "FLASH" {
            continue;
        }
        let origin = region_field(rest, "ORIGIN")
            .unwrap_or_else(|| panic!("bad FLASH ORIGIN in {path}"));
        let length = region_field(rest, "LENGTH")
            .unwrap_or_else(|| panic!("bad FLASH LENGTH in {path}"));
        return (origin, length);
    }
    panic!("no FLASH region in {path}");
}

fn region_field(s: &str, key: &str) -> Option<u32> {
    let after = s.split_once(key)?.1;
    let value = after.trim_start().strip_prefix('=')?.trim_start();
    let end = value
        .find(|c: char| c == ',' || c.is_whitespace())
        .unwrap_or(value.len());
    linker_number(&value[..end])
}

/// Linker script numbers: hex, decimal, and K/M suffixes
fn linker_number(s: &str) -> Option<u32> {
    if let Some(hex) = s.strip_prefix("0x") {
        return u32::from_str_radix(hex, 16).ok();
    }
    if let Some(k) = s.strip_suffix('K') {
        return k.parse::<u32>().ok().map(|v| v * 1024);
    }
    if let Some(m) = s.strip_suffix('M') {
        return m.parse::<u32>().ok().map(|v| v * 1024 * 1024);
    }
    s.parse().ok()
}
