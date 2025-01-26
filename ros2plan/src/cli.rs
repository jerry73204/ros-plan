use clap::Parser;
use std::path::PathBuf;

#[derive(Debug, Clone, Parser)]
pub enum Cli {
    Start(StartArgs),
    Expand(ExpandArgs),
    Load(LoadArgs),
}

#[derive(Debug, Clone, Parser)]
pub struct StartArgs {
    /// The ROS plan file.
    pub plan_file: PathBuf,

    /// The execution environment configuration file.
    #[clap(short = 'c', long)]
    pub config: Option<PathBuf>,
}

#[derive(Debug, Clone, Parser)]
pub struct ExpandArgs {
    /// The ROS plan file.
    pub plan_file: PathBuf,

    /// The path to the output file.
    #[clap(short = 'o', long)]
    pub output_file: Option<PathBuf>,
}

#[derive(Debug, Clone, Parser)]
pub struct DumpArgs {
    /// The ROS plan file.
    pub plan_file: PathBuf,

    /// The path to the output file.
    #[clap(short = 'o', long)]
    pub output_file: Option<PathBuf>,
}

#[derive(Debug, Clone, Parser)]
pub struct LoadArgs {
    /// The compiled ROS plan file.
    pub dump_file: PathBuf,
}
