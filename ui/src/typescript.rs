//! Scenario scripts in TypeScript: their types stripped by oxc before QuickJS
//! runs them, and the positions errors name mapped back to the script.
//!
//! Stripping reprints the program, so QuickJS sees other lines and columns
//! than the script has; the source map oxc makes alongside maps each position
//! an error names back to where it was in the script.

use std::path::Path;

use oxc::allocator::Allocator;
use oxc::codegen::{Codegen, CodegenOptions};
use oxc::parser::Parser;
use oxc::semantic::SemanticBuilder;
use oxc::span::SourceType;
use oxc::transformer::{TransformOptions, Transformer};

/// A TypeScript script as JavaScript, with how to map positions back.
pub struct Stripped {
    pub code: String,
    /// The source map's tokens, each a position in the code and where it was
    /// in the script, as `[line, column]`s from 0, in order of the first.
    map: Vec<[[u32; 2]; 2]>,
}

/// `source`, a TypeScript script named `file`, with its types stripped, or
/// its syntax errors, each with where it is.
pub fn strip(source: &str, file: &str) -> Result<Stripped, String> {
    let allocator = Allocator::default();
    let parsed = Parser::new(&allocator, source, SourceType::ts()).parse();
    if !parsed.diagnostics.is_empty() {
        return Err(describe(source, file, &parsed.diagnostics));
    }
    let mut program = parsed.program;
    let scoping = SemanticBuilder::new().build(&program).semantic.into_scoping();
    let path = Path::new(file);
    let transformed = Transformer::new(&allocator, path, &TransformOptions::default())
        .build_with_scoping(scoping, &mut program);
    if !transformed.diagnostics.is_empty() {
        return Err(describe(source, file, &transformed.diagnostics));
    }
    let options = CodegenOptions {
        source_map_path: Some(path.to_path_buf()),
        ..Default::default()
    };
    let generated = Codegen::new().with_options(options).build(&program);
    let map = generated.map.expect("asked for a source map");
    let mut map: Vec<[[u32; 2]; 2]> = map
        .get_tokens()
        .map(|t| {
            [
                [t.get_dst_line(), t.get_dst_col()],
                [t.get_src_line(), t.get_src_col()],
            ]
        })
        .collect();
    map.sort();
    Ok(Stripped { code: generated.code, map })
}

impl Stripped {
    /// `message`, an error from running the stripped script named `file`,
    /// with each `file:line:column` it names moved to where that is in the
    /// script.
    pub fn map_error(&self, file: &str, message: &str) -> String {
        let prefix = format!("{file}:");
        let mut out = String::new();
        let mut rest = message;
        while let Some(at) = rest.find(&prefix) {
            let (before, after) = rest.split_at(at + prefix.len());
            out.push_str(before);
            rest = after;
            // `line:column`, both from 1, if that is what follows.
            let digits = |s: &str| s.find(|c: char| !c.is_ascii_digit()).unwrap_or(s.len());
            let line_len = digits(rest);
            let Some(col_text) = rest[line_len..].strip_prefix(':') else {
                continue;
            };
            let col_len = digits(col_text);
            let (Ok(line), Ok(col)) = (
                rest[..line_len].parse::<u32>(),
                col_text[..col_len].parse::<u32>(),
            ) else {
                continue;
            };
            if let Some([line, col]) =
                self.source_position([line.saturating_sub(1), col.saturating_sub(1)])
            {
                out.push_str(&format!("{}:{}", line + 1, col + 1));
                rest = &col_text[col_len..];
            }
        }
        out.push_str(rest);
        out
    }

    /// Where `at`, a position in the code, was in the script: from the last
    /// token on its line at or before it, or failing that the first on its
    /// line.
    fn source_position(&self, at: [u32; 2]) -> Option<[u32; 2]> {
        let line = |t: &[[u32; 2]; 2]| t[0][0];
        let on_line: Vec<_> = self.map.iter().filter(|t| line(t) == at[0]).collect();
        let before = on_line.iter().rev().find(|t| t[0][1] <= at[1]);
        before.or(on_line.first()).map(|t| t[1])
    }
}

/// Diagnostics as `file:line:column: message` lines.
fn describe(source: &str, file: &str, diagnostics: &[oxc::diagnostics::OxcDiagnostic]) -> String {
    let lines: Vec<String> = diagnostics
        .iter()
        .map(|d| match d.labels.first() {
            Some(label) => {
                let before = &source[..(label.offset() as usize).min(source.len())];
                let line = before.matches('\n').count() + 1;
                let col = before.len() - before.rfind('\n').map_or(0, |i| i + 1) + 1;
                format!("{file}:{line}:{col}: {}", d.message)
            }
            None => format!("{file}: {}", d.message),
        })
        .collect();
    lines.join("\n")
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn types_are_stripped() {
        let stripped = strip(
            "type Side = 1 | -1;\nconst f = (side: Side): number => side * 2;\nconst x = f(1 as Side);",
            "t.ts",
        )
        .unwrap();
        assert!(!stripped.code.contains("Side"), "{}", stripped.code);
        assert!(stripped.code.contains("side * 2"), "{}", stripped.code);
    }

    #[test]
    fn syntax_errors_say_where_they_are() {
        let error = strip("const a = 1;\nconst b: = 2;", "bad.ts").err().unwrap();
        assert!(error.starts_with("bad.ts:2:"), "{error}");
    }

    #[test]
    fn positions_in_errors_map_back_to_the_script() {
        // Stripping moves the call up a line, past the interface.
        let source = "interface A {\n  x: number;\n}\nconst a: A = { x: 1 };\nundefined_thing();\n";
        let stripped = strip(source, "s.ts").unwrap();
        let line = stripped.code.lines().position(|l| l.contains("undefined_thing")).unwrap() + 1;
        let message = format!("ReferenceError\n    at <eval> (s.ts:{line}:1)");
        assert_eq!(
            stripped.map_error("s.ts", &message),
            "ReferenceError\n    at <eval> (s.ts:5:1)"
        );
    }
}
