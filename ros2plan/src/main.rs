pub mod config;

use clap::Parser;
use indexmap::IndexMap;
use std::path::PathBuf;

#[derive(Debug, Clone, Parser)]
struct Args {
    /// The ROS plan file.
    pub plan_file: PathBuf,

    /// The execution environment configuration file.
    #[clap(short = 'c', long)]
    pub config: Option<PathBuf>,
}

fn main() -> eyre::Result<()> {
    let args = Args::parse();
    let mut resource = ros_plan_parse::compile_plan_file(&args.plan_file)?;
    ros_plan_parse::eval_resource(&mut resource, IndexMap::new())?;

    let json_text = serde_json::to_string_pretty(&resource)?;
    println!("{json_text}");

    Ok(())
}
