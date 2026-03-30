//! Vistio CLI — simulation, benchmarking, and debugging.

use clap::{Parser, Subcommand};

mod commands;

#[derive(Parser)]
#[command(name = "vistio")]
#[command(version, about = "Vistio — GPU-accelerated garment simulation engine")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Run a simulation from a config file.
    Simulate {
        /// Path to simulation config (TOML).
        #[arg(short, long, default_value = "simulation.toml")]
        config: String,
    },

    /// Run benchmark suite.
    Benchmark {
        /// Which scenario to run (hanging_sheet, sphere_drape, all).
        #[arg(short, long, default_value = "all")]
        scenario: String,

        /// Output CSV file path.
        #[arg(short, long)]
        output: Option<String>,

        /// Material name from the database (e.g., cotton_twill, silk_charmeuse, denim_14oz).
        /// When set, simulation uses material-aware initialization.
        #[arg(short, long)]
        material: Option<String>,
    },

    /// Run a simulation and export mesh frames for visual inspection.
    Visualize {
        /// Which scenario to run.
        #[arg(short, long, default_value = "hanging_sheet")]
        scenario: String,

        /// Material name from database.
        #[arg(short, long)]
        material: Option<String>,

        /// Output JSON file path.
        #[arg(short, long, default_value = "simulation.json")]
        output: String,
    },

    /// Inspect a state snapshot file.
    Inspect {
        /// Path to snapshot file.
        path: String,
    },

    /// Validate a mesh or simulation input.
    Validate {
        /// Path to mesh or config file.
        path: String,
    },
}

fn main() {
    let cli = Cli::parse();

    let result = match cli.command {
        Commands::Simulate { config } => commands::simulate(&config),
        Commands::Benchmark { scenario, output, material } => commands::benchmark(&scenario, output.as_deref(), material.as_deref()),
        Commands::Visualize { scenario, material, output } => commands::visualize(&scenario, material.as_deref(), &output),
        Commands::Inspect { path } => commands::inspect(&path),
        Commands::Validate { path } => commands::validate(&path),
    };

    if let Err(e) = result {
        eprintln!("Error: {e}");
        std::process::exit(1);
    }
}
