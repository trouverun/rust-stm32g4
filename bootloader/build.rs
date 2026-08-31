use std::env;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let board = board_name();
    let memory_x = format!("memory/{board}.x");
    std::fs::copy(&memory_x, out.join("memory.x"))
        .unwrap_or_else(|e| panic!("copy {memory_x}: {e}"));
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed={memory_x}");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    generate_memory_layout(out, &board);
}

/// The single enabled `board-*` cargo feature
fn board_name() -> String {
    let boards: Vec<String> = env::vars()
        .filter_map(|(k, _)| k.strip_prefix("CARGO_FEATURE_BOARD_").map(|b| b.to_lowercase()))
        .collect();
    match boards.as_slice() {
        [board] => board.clone(),
        [] => panic!("no board selected: build with --features board-<name>"),
        _ => panic!("multiple board features enabled: {boards:?}"),
    }
}

/// Emits FIRMWARE_ORIGIN and FIRMWARE_SIZE from the two memory.x files
fn generate_memory_layout(out_dir: &PathBuf, board: &str) {
    let firmware_memory_x = format!("../firmware/memory/{board}.x");
    println!("cargo:rerun-if-changed={firmware_memory_x}");
    let (origin, length) = flash_region(&format!("memory/{board}.x"));
    let (_, firmware_size) = flash_region(&firmware_memory_x);
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
