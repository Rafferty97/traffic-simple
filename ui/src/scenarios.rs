//! The scenarios to choose from: the scripts in the repository's `scenarios`
//! directory, in JavaScript or TypeScript, each named by its first line if that is `// name: …`, or by its
//! file name. Only names and paths are kept: a script is read afresh each time
//! it is loaded.

use std::path::{Path, PathBuf};

/// Where the scenarios are.
pub const DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../scenarios");

/// A scenario's script.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ScenarioFile {
    pub name: String,
    pub path: PathBuf,
}

impl ScenarioFile {
    /// Its file name without its extension, by which the command line names
    /// it.
    pub fn stem(&self) -> &str {
        self.path.file_stem().and_then(|s| s.to_str()).unwrap_or_default()
    }
}

/// The scripts in `dir`, by file name, or why they couldn't be listed.
pub fn list(dir: &Path) -> Result<Vec<ScenarioFile>, String> {
    let entries = std::fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let mut files: Vec<ScenarioFile> = entries
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| is_script(path) && path.is_file())
        .map(|path| {
            let file_name = path.file_name().unwrap_or_default().to_string_lossy().into_owned();
            let source = std::fs::read_to_string(&path).unwrap_or_default();
            let name = name(&source).unwrap_or(file_name);
            ScenarioFile { name, path }
        })
        .collect();
    files.sort_by(|a, b| a.path.cmp(&b.path));
    Ok(files)
}

/// Whether `path` is a script: JavaScript or TypeScript, but not
/// declarations.
fn is_script(path: &Path) -> bool {
    let name = path.file_name().and_then(|n| n.to_str()).unwrap_or_default();
    (name.ends_with(".js") || name.ends_with(".ts")) && !name.ends_with(".d.ts")
}

/// The name a script gives itself on its first line, `// name: …`.
fn name(source: &str) -> Option<String> {
    let first = source.lines().next()?;
    let name = first.strip_prefix("// name:")?.trim();
    (!name.is_empty()).then(|| name.to_string())
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn a_script_s_first_line_names_it() {
        assert_eq!(
            name("// name: A straight road\nconst x = 1;"),
            Some("A straight road".into())
        );
        assert_eq!(name("const x = 1;\n// name: Not here"), None);
        assert_eq!(name("// name:   \n"), None);
        assert_eq!(name(""), None);
    }

    #[test]
    fn the_repository_s_scenarios_are_listed() {
        let files = list(Path::new(DIR)).unwrap();
        assert!(!files.is_empty());
        assert!(files.iter().all(|f| is_script(&f.path)));
        assert!(
            !files.iter().any(|f| f.stem() == "api.d"),
            "declarations aren't scenarios"
        );
    }
}
