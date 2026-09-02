use std::collections::HashMap;
use std::env;
use std::fmt::Write as _;

use can_dbc::{
    AttributeValue, AttributeValuedForObjectType, DBC, Message, MultiplexIndicator,
    Signal, Transmitter, ValueType,
};
use heck::{ToPascalCase, ToSnakeCase};

const CYCLE_TIME_MS_ATTRIBUTE: &str = "GenMsgCycleTime";
const CYCLE_TIME_US_ATTRIBUTE: &str = "GenMsgCycleTimeUs";
/// RAM reserved for core firmware use 
/// (with remainder being free to use for the debug capture buffer feature)
const RESERVED_RAM_BYTES: u32 = 40 * 1024;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    let out_dir = env::var("OUT_DIR").unwrap();
    let board = board_name();
    check_encoder();

    configure_linker(&out_dir, &board);
    configure_defmt();
    generate_memory_layout(&out_dir, &board);
    generate_version(&out_dir);
    generate_can(&out_dir);
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

/// At most one `encoder-*` cargo feature, emits the `encoder_none` cfg when there is none
fn check_encoder() {
    println!("cargo:rustc-check-cfg=cfg(encoder_none)");
    let encoders: Vec<String> = env::vars()
        .filter_map(|(k, _)| k.strip_prefix("CARGO_FEATURE_ENCODER_").map(|e| e.to_lowercase()))
        .collect();
    match encoders.as_slice() {
        [_] => {}
        [] => println!("cargo:rustc-cfg=encoder_none"),
        _ => panic!("multiple encoder features enabled: {encoders:?}"),
    }
}

/// Emits VERSION_MAJOR/MINOR/PATCH as u8 from the package version.
fn generate_version(out_dir: &str) {
    let mut src = String::new();
    for part in ["MAJOR", "MINOR", "PATCH"] {
        let v: u8 = env::var(format!("CARGO_PKG_VERSION_{part}")).unwrap().parse().unwrap();
        writeln!(src, "pub(crate) const VERSION_{part}: u8 = {v};").unwrap();
    }
    std::fs::write(std::path::Path::new(out_dir).join("version.rs"), src)
        .expect("write version.rs");
}

fn configure_linker(out_dir: &str, board: &str) {
    let memory_x = format!("memory/{board}.x");
    std::fs::copy(&memory_x, std::path::Path::new(out_dir).join("memory.x"))
        .unwrap_or_else(|e| panic!("copy {memory_x}: {e}"));
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    println!("cargo:rustc-link-arg-bins=-Tccmram.x");
    println!("cargo:rustc-link-search={out_dir}");
    println!("cargo:rustc-link-search={}", env::var("CARGO_MANIFEST_DIR").unwrap());
    println!("cargo:rerun-if-changed={memory_x}");
    println!("cargo:rerun-if-changed=ccmram.x");
}

fn configure_defmt() {
    match env::var("PROFILE").unwrap().as_str() {
        "release" => println!("cargo:rustc-env=DEFMT_LOG=error"),
        _ => println!("cargo:rustc-env=DEFMT_LOG=trace"),
    }
}

/// Emits FIRMWARE_SIZE and CAPTURE_RAM_BYTES from memory.x and checks that the bootloader's
/// flash ends exactly where the firmware's begins.
fn generate_memory_layout(out_dir: &str, board: &str) {
    let loader_memory_x = format!("../bootloader/memory/{board}.x");
    println!("cargo:rerun-if-changed={loader_memory_x}");
    let memory_x = format!("memory/{board}.x");
    let (firmware_origin, firmware_size) = region(&memory_x, "FLASH");
    let (_, ram_size) = region(&memory_x, "RAM");
    let (loader_origin, loader_size) = region(&loader_memory_x, "FLASH");
    assert!(
        loader_origin + loader_size == firmware_origin,
        "bootloader flash ends at {:#x} but firmware starts at {:#x}",
        loader_origin + loader_size,
        firmware_origin,
    );
    let capture_ram = ram_size.checked_sub(RESERVED_RAM_BYTES).unwrap_or_else(|| {
        panic!("RAM is {ram_size:#x} bytes but {RESERVED_RAM_BYTES:#x} are reserved")
    });
    let layout = format!(
        "pub(crate) const FIRMWARE_SIZE: u32 = {firmware_size:#x};\npub(crate) const CAPTURE_RAM_BYTES: u32 = {capture_ram:#x};\n"
    );
    std::fs::write(std::path::Path::new(out_dir).join("layout.rs"), layout)
        .expect("write layout.rs");
}

/// ORIGIN and LENGTH of a memory region in a linker memory script
fn region(path: &str, region: &str) -> (u32, u32) {
    let src = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    for line in src.lines() {
        let line = line.split("/*").next().unwrap();
        let Some((name, rest)) = line.split_once(':') else { continue };
        if name.trim() != region {
            continue;
        }
        let origin = region_field(rest, "ORIGIN")
            .unwrap_or_else(|| panic!("bad {region} ORIGIN in {path}"));
        let length = region_field(rest, "LENGTH")
            .unwrap_or_else(|| panic!("bad {region} LENGTH in {path}"));
        return (origin, length);
    }
    panic!("no {region} region in {path}");
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

fn generate_can(out_dir: &str) {
    println!("cargo:rerun-if-changed=dbc/can.dbc");

    let dbc_path = "dbc/can.dbc";
    let dbc_bytes = std::fs::read(dbc_path).expect("read dbc");
    let dbc = DBC::from_slice(&dbc_bytes).expect("parse dbc");

    let messages_path = std::path::Path::new(&out_dir).join("messages.rs");
    let periodic_path = std::path::Path::new(&out_dir).join("periodic.rs");
    let frames_path = std::path::Path::new(&out_dir).join("frames.rs");

    let mut buf: Vec<u8> = Vec::new();
    dbc_codegen::codegen("can.dbc", &dbc_bytes, &mut buf, true)
        .expect("dbc codegen");
    let raw = String::from_utf8(buf).expect("dbc codegen produced non-utf8 output");

    // dbc-codegen emits inner attributes (`#![...]`) and inner doc comments (`//!`)
    // intended for a top-level file. We `include!` the result inside an existing
    // module, where inner attributes are not permitted, so strip the leading run.
    let mut messages_src: String = raw
        .lines()
        .skip_while(|l| {
            let t = l.trim();
            t.is_empty() || t.starts_with("#!") || t.starts_with("//")
        })
        .collect::<Vec<_>>()
        .join("\n");

    // dbc-codegen setters truncate the scaled raw value, so the f32 factor
    // round-trip (stored value ends up a hair below the raw grid point) makes
    // reports read back one unit of resolution low. Round to nearest instead.
    messages_src = messages_src.replace(
        "((value - offset) / factor) as ",
        "libm::roundf((value - offset) / factor) as ",
    );

    let cycle_times = collect_cycle_times(&dbc);

    messages_src.push_str("\n\n// ---- generated by build.rs (Init structs + cycle times) ----\n");
    for msg in dbc.messages().iter().filter(|m| is_board_tx(m)) {
        emit_init_struct(&mut messages_src, msg);
        if let Some(us) = cycle_times.get(&msg.message_id().0).copied() {
            if us > 0 {
                emit_cycle_time_const(&mut messages_src, msg, us);
            }
        }
    }

    std::fs::write(&messages_path, messages_src).expect("write messages.rs");

    let periodic_src = emit_periodic(&dbc, &cycle_times);
    std::fs::write(&periodic_path, periodic_src).expect("write periodic.rs");

    let frames_src = emit_frames(&dbc);
    std::fs::write(&frames_path, frames_src).expect("write frames.rs");
}

/// Returns each periodic message's transmit period in **microseconds**.
///
/// Uses `GenMsgCycleTimeUs` when set, otherwise falls back to
/// `GenMsgCycleTime` (milliseconds, scaled to µs).
fn collect_cycle_times(dbc: &DBC) -> HashMap<u32, u32> {
    let mut out = HashMap::new();
    for (id, ms) in collect_int_attribute(dbc, CYCLE_TIME_MS_ATTRIBUTE) {
        out.insert(id, ms.saturating_mul(1000));
    }
    for (id, us) in collect_int_attribute(dbc, CYCLE_TIME_US_ATTRIBUTE) {
        out.insert(id, us); // microsecond attribute overrides millisecond
    }
    out
}

/// Collects a non-negative integer message attribute into `id -> value`.
/// Only explicitly set (`BA_`) values are returned, never definition defaults.
fn collect_int_attribute(dbc: &DBC, name: &str) -> HashMap<u32, u32> {
    let mut out = HashMap::new();
    for av in dbc.attribute_values() {
        if av.attribute_name() != name {
            continue;
        }
        let AttributeValuedForObjectType::MessageDefinitionAttributeValue(id, value) =
            av.attribute_value()
        else {
            continue;
        };
        let v = match value {
            Some(AttributeValue::AttributeValueU64(n)) => *n as u32,
            Some(AttributeValue::AttributeValueI64(n)) if *n >= 0 => *n as u32,
            Some(AttributeValue::AttributeValueF64(n)) if *n >= 0.0 => *n as u32,
            _ => continue,
        };
        out.insert(id.0, v);
    }
    out
}

fn emit_init_struct(out: &mut String, msg: &Message) {
    let msg_type = type_name(msg.message_name());
    let fields: Vec<(String, String)> = msg
        .signals()
        .iter()
        .filter(|sig| is_constructor_signal(sig))
        .map(|sig| (field_name(sig.name()), signal_to_rust_type(sig)))
        .collect();

    writeln!(out, "\npub struct {msg_type}Init {{").unwrap();
    for (name, ty) in &fields {
        writeln!(out, "    pub {name}: {ty},").unwrap();
    }
    writeln!(out, "}}").unwrap();

    writeln!(out, "\nimpl core::convert::TryFrom<{msg_type}Init> for {msg_type} {{").unwrap();
    writeln!(out, "    type Error = CanError;").unwrap();
    writeln!(out, "    fn try_from(v: {msg_type}Init) -> Result<Self, CanError> {{").unwrap();
    let args: Vec<String> = fields.iter().map(|(n, _)| format!("v.{n}")).collect();
    writeln!(out, "        Self::new({})", args.join(", ")).unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out, "}}").unwrap();
}

fn emit_cycle_time_const(out: &mut String, msg: &Message, us: u32) {
    let msg_type = type_name(msg.message_name());
    writeln!(out, "\nimpl {msg_type} {{ pub const CYCLE_TIME_US: u32 = {us}; }}").unwrap();
}

fn emit_frames(dbc: &DBC) -> String {
    let mut s = String::from("// AUTOGENERATED, do not edit\n\n");
    for msg in dbc.messages().iter().filter(|m| is_board_tx(m)) {
        let msg_type = type_name(msg.message_name());
        writeln!(s, "impl IntoFrame for {msg_type} {{").unwrap();
        writeln!(s, "    fn into_frame(&self) -> Frame {{").unwrap();
        writeln!(
            s,
            "        Frame::new_standard({msg_type}::MESSAGE_ID as u16, self.raw()).unwrap()"
        )
        .unwrap();
        writeln!(s, "    }}").unwrap();
        writeln!(s, "}}\n").unwrap();
    }
    s
}

fn emit_periodic(dbc: &DBC, cycle_times: &HashMap<u32, u32>) -> String {
    let periodic: Vec<&Message> = dbc
        .messages()
        .iter()
        .filter(|m| cycle_times.get(&m.message_id().0).copied().unwrap_or(0) > 0)
        .collect();

    let mut s = String::from("// AUTOGENERATED, do not edit\n\n");
    writeln!(s, "#[derive(Clone, Copy)]").unwrap();
    writeln!(s, "pub enum Periodic {{").unwrap();
    for m in &periodic {
        writeln!(s, "    {},", type_name(m.message_name())).unwrap();
    }
    writeln!(s, "}}").unwrap();

    writeln!(s, "\nimpl Periodic {{").unwrap();
    writeln!(s, "    pub const COUNT: usize = {};", periodic.len()).unwrap();
    writeln!(s, "\n    pub const fn all() -> [Self; Self::COUNT] {{").unwrap();
    write!(s, "        [").unwrap();
    for (i, m) in periodic.iter().enumerate() {
        if i > 0 {
            write!(s, ", ").unwrap();
        }
        write!(s, "Periodic::{}", type_name(m.message_name())).unwrap();
    }
    writeln!(s, "]").unwrap();
    writeln!(s, "    }}").unwrap();

    writeln!(s, "\n    pub const fn period_us(self) -> u32 {{").unwrap();
    writeln!(s, "        match self {{").unwrap();
    for m in &periodic {
        let t = type_name(m.message_name());
        writeln!(s, "            Periodic::{t} => {t}::CYCLE_TIME_US,").unwrap();
    }
    writeln!(s, "        }}").unwrap();
    writeln!(s, "    }}").unwrap();
    writeln!(s, "}}").unwrap();

    s
}

// TX helpers (Init/IntoFrame/periodic) only apply to messages the board sends.
// Host-sent messages are decoded via dbc-codegen's generated `Messages`.
fn is_board_tx(msg: &Message) -> bool {
    matches!(msg.transmitter(), Transmitter::NodeName(n) if n == "Firmware")
}

fn is_constructor_signal(sig: &Signal) -> bool {
    matches!(
        sig.multiplexer_indicator(),
        MultiplexIndicator::Plain | MultiplexIndicator::Multiplexor
    )
}

// Mirrors dbc-codegen's signal_to_rust_type so field types line up with
// the positional `Self::new(...)` signature.
fn signal_to_rust_type(sig: &Signal) -> String {
    if sig.signal_size == 1 {
        "bool".into()
    } else if sig.offset != 0.0 || sig.factor != 1.0 {
        "f32".into()
    } else {
        let sign = match sig.value_type() {
            ValueType::Signed => "i",
            ValueType::Unsigned => "u",
        };
        let bits = match sig.signal_size {
            n if n <= 8 => "8",
            n if n <= 16 => "16",
            n if n <= 32 => "32",
            _ => "64",
        };
        format!("{sign}{bits}")
    }
}

// Mirrors dbc-codegen's field_name.
fn field_name(name: &str) -> String {
    if is_rust_keyword(name) || !name.starts_with(|c: char| c.is_ascii_alphabetic()) {
        format!("x{}", name.to_snake_case())
    } else {
        name.to_snake_case()
    }
}

// Mirrors dbc-codegen's type_name.
fn type_name(name: &str) -> String {
    if is_rust_keyword(name) || !name.starts_with(|c: char| c.is_ascii_alphabetic()) {
        format!("X{}", name.to_pascal_case())
    } else {
        name.to_pascal_case()
    }
}

fn is_rust_keyword(s: &str) -> bool {
    matches!(
        s,
        "as" | "break" | "const" | "continue" | "crate" | "else" | "enum" | "extern"
        | "false" | "fn" | "for" | "if" | "impl" | "in" | "let" | "loop" | "match"
        | "mod" | "move" | "mut" | "pub" | "ref" | "return" | "self" | "Self" | "static"
        | "struct" | "super" | "trait" | "true" | "type" | "unsafe" | "use" | "where"
        | "while" | "async" | "await" | "dyn" | "abstract" | "become" | "box" | "do"
        | "final" | "macro" | "override" | "priv" | "typeof" | "unsized" | "virtual"
        | "yield" | "try" | "union"
    )
}
