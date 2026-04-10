//! play_launch_parser CLI

use clap::{Parser, Subcommand, ValueEnum};
use play_launch_parser::{parse_launch_file, record::RecordJson};
use std::{
    collections::HashMap,
    io::{self, Write},
    path::PathBuf,
    process,
};

#[derive(Parser)]
#[command(name = "play_launch_parser")]
#[command(about = "High-performance ROS 2 launch file parser", long_about = None)]
#[command(version)]
struct Cli {
    #[command(subcommand)]
    command: Commands,

    /// Enable debug logging
    #[arg(short, long, global = true)]
    verbose: bool,

    /// Suppress all log output
    #[arg(short, long, global = true)]
    quiet: bool,

    /// Output format
    #[arg(short, long, value_enum, default_value_t = Format::Json, global = true)]
    format: Format,

    /// Write output to file instead of stdout
    #[arg(short, long, global = true)]
    output: Option<PathBuf>,
}

#[derive(Clone, ValueEnum)]
enum Format {
    /// Full record.json output
    Json,
    /// Human-readable summary table
    Summary,
    /// One node FQN per line
    Names,
}

#[derive(Subcommand)]
enum Commands {
    /// Parse a launch file from a ROS 2 package
    Launch {
        /// Package name
        package: String,

        /// Launch file name
        file: String,

        /// Launch arguments (key:=value)
        #[arg(value_parser = parse_launch_arg)]
        args: Vec<(String, String)>,
    },

    /// Parse a launch file from a direct file path
    File {
        /// Launch file path
        path: PathBuf,

        /// Launch arguments (key:=value)
        #[arg(value_parser = parse_launch_arg)]
        args: Vec<(String, String)>,
    },
}

fn parse_launch_arg(s: &str) -> Result<(String, String), String> {
    let parts: Vec<&str> = s.split(":=").collect();
    if parts.len() != 2 {
        return Err(format!("Invalid launch argument format: {}", s));
    }
    Ok((parts[0].to_string(), parts[1].to_string()))
}

fn find_launch_file(package: &str, file: &str) -> Result<PathBuf, String> {
    if let Ok(ament_paths) = std::env::var("AMENT_PREFIX_PATH") {
        for prefix in ament_paths.split(':') {
            let path = PathBuf::from(prefix)
                .join("share")
                .join(package)
                .join("launch")
                .join(file);
            if path.exists() {
                return Ok(path);
            }
        }
    }
    Err(format!(
        "Launch file not found: {} in package {}",
        file, package
    ))
}

/// Build a fully-qualified node name from namespace + name fields.
fn node_fqn(namespace: Option<&str>, name: Option<&str>) -> String {
    let ns = namespace.unwrap_or("/");
    let n = name.unwrap_or("?");
    if ns == "/" {
        format!("/{n}")
    } else {
        format!("{ns}/{n}")
    }
}

/// Format the record as a human-readable summary.
fn format_summary(record: &RecordJson, launch_label: &str) -> String {
    let mut out = String::new();
    out.push_str(&format!("Launch: {launch_label}\n\n"));
    out.push_str(&format!("  Nodes:        {}\n", record.node.len()));
    out.push_str(&format!("  Containers:   {}\n", record.container.len()));
    out.push_str(&format!("  Composable:   {}\n", record.load_node.len()));
    out.push_str(&format!("  Scopes:       {}\n", record.scopes.len()));
    out.push_str(&format!("  Variables:    {}\n", record.variables.len()));

    if !record.node.is_empty() {
        out.push_str("\nNodes:\n");
        for node in &record.node {
            let fqn = node_fqn(node.namespace.as_deref(), node.name.as_deref());
            let pkg = node.package.as_deref().unwrap_or("-");
            out.push_str(&format!("  {fqn:<60} {pkg}\n"));
        }
    }

    if !record.container.is_empty() {
        out.push_str("\nContainers:\n");
        for c in &record.container {
            let fqn = node_fqn(Some(&c.namespace), Some(&c.name));
            out.push_str(&format!("  {fqn:<60} {}\n", c.package));
        }
    }

    if !record.load_node.is_empty() {
        out.push_str("\nComposable Nodes:\n");
        for ln in &record.load_node {
            let fqn = node_fqn(Some(&ln.namespace), Some(&ln.node_name));
            out.push_str(&format!(
                "  {fqn:<60} {} -> {}\n",
                ln.plugin, ln.target_container_name
            ));
        }
    }

    out
}

/// Format the record as one FQN per line.
fn format_names(record: &RecordJson) -> String {
    let mut out = String::new();
    for node in &record.node {
        out.push_str(&node_fqn(node.namespace.as_deref(), node.name.as_deref()));
        out.push('\n');
    }
    for c in &record.container {
        out.push_str(&node_fqn(Some(&c.namespace), Some(&c.name)));
        out.push('\n');
    }
    for ln in &record.load_node {
        out.push_str(&node_fqn(Some(&ln.namespace), Some(&ln.node_name)));
        out.push('\n');
    }
    out
}

fn main() {
    let cli = Cli::parse();

    // Set up logging — always to stderr
    let log_level = if cli.verbose {
        "debug"
    } else if cli.quiet {
        "error"
    } else {
        "info"
    };
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(log_level))
        .target(env_logger::Target::Stderr)
        .init();

    // Resolve launch file path
    let (launch_path, launch_label) = match &cli.command {
        Commands::Launch {
            package,
            file,
            args: _,
        } => {
            log::info!("Parsing launch file: {} from package {}", file, package);
            match find_launch_file(package, file) {
                Ok(path) => {
                    let label = format!("{file} ({package})");
                    (path, label)
                }
                Err(e) => {
                    eprintln!("Error: {}", e);
                    process::exit(2);
                }
            }
        }
        Commands::File { path, args: _ } => {
            log::info!("Parsing launch file: {}", path.display());
            if !path.exists() {
                eprintln!("Error: file not found: {}", path.display());
                process::exit(2);
            }
            let label = path
                .file_name()
                .and_then(|s| s.to_str())
                .unwrap_or("unknown")
                .to_string();
            (path.clone(), label)
        }
    };

    // Collect args
    let cli_args: HashMap<String, String> = match &cli.command {
        Commands::Launch { args, .. } | Commands::File { args, .. } => {
            args.iter().cloned().collect()
        }
    };

    // Parse
    let record = match parse_launch_file(&launch_path, cli_args) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("Error: {}", e);
            process::exit(1);
        }
    };

    // Format output
    let output_text = match cli.format {
        Format::Json => match record.to_json() {
            Ok(json) => json,
            Err(e) => {
                eprintln!("Error: {}", e);
                process::exit(1);
            }
        },
        Format::Summary => format_summary(&record, &launch_label),
        Format::Names => format_names(&record),
    };

    // Write output
    if let Some(output_path) = &cli.output {
        if let Err(e) = std::fs::write(output_path, &output_text) {
            eprintln!("Error writing {}: {}", output_path.display(), e);
            process::exit(1);
        }
        log::info!("Wrote output to {}", output_path.display());
    } else {
        let stdout = io::stdout();
        let mut handle = stdout.lock();
        if let Err(e) = handle.write_all(output_text.as_bytes()) {
            // Broken pipe is expected when piping to head/grep
            if e.kind() != io::ErrorKind::BrokenPipe {
                eprintln!("Error: {}", e);
                process::exit(1);
            }
        }
    }
}
