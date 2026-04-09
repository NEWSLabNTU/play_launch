//! play_launch_parser CLI

use clap::{Parser, Subcommand};
use play_launch_parser::parse_launch_file;
use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    process,
};

#[derive(Parser)]
#[command(name = "play_launch_parser")]
#[command(about = "High-performance ROS 2 launch file parser", long_about = None)]
#[command(version)]
struct Cli {
    #[command(subcommand)]
    command: Commands,

    #[arg(short, long)]
    verbose: bool,

    #[arg(short, long)]
    quiet: bool,
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

        /// Output file path (default: record.json)
        #[arg(short, long, default_value = "record.json")]
        output: PathBuf,
    },

    /// Parse a launch file from a direct file path
    File {
        /// Launch file path
        path: PathBuf,

        /// Launch arguments (key:=value)
        #[arg(value_parser = parse_launch_arg)]
        args: Vec<(String, String)>,

        /// Output file path (default: record.json)
        #[arg(short, long, default_value = "record.json")]
        output: PathBuf,
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

fn main() {
    let cli = Cli::parse();

    // Set up logging
    let log_level = if cli.verbose {
        "debug"
    } else if cli.quiet {
        "error"
    } else {
        "info"
    };
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(log_level)).init();

    let result = match cli.command {
        Commands::Launch {
            package,
            file,
            args,
            output,
        } => {
            log::info!("Parsing launch file: {} from package {}", file, package);
            let launch_path = match find_launch_file(&package, &file) {
                Ok(path) => path,
                Err(e) => {
                    eprintln!("Error: {}", e);
                    process::exit(1);
                }
            };

            let cli_args: HashMap<String, String> = args.into_iter().collect();
            parse_and_write(&launch_path, cli_args, &output)
        }
        Commands::File { path, args, output } => {
            log::info!("Parsing launch file: {}", path.display());
            let cli_args: HashMap<String, String> = args.into_iter().collect();
            parse_and_write(&path, cli_args, &output)
        }
    };

    if let Err(e) = result {
        eprintln!("Error: {}", e);
        process::exit(1);
    }
}

fn parse_and_write(
    launch_path: &Path,
    cli_args: HashMap<String, String>,
    output: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    // Parse launch file
    let record = parse_launch_file(launch_path, cli_args)?;

    // Write to output file
    let json = record.to_json()?;
    std::fs::write(output, json)?;

    log::info!("Generated record.json: {}", output.display());
    log::info!(
        "  {} nodes, {} containers, {} composable nodes",
        record.node.len(),
        record.container.len(),
        record.load_node.len()
    );

    Ok(())
}
