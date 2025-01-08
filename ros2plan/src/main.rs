use std::path::PathBuf;

use clap::Parser;

#[derive(Debug, Clone, Parser)]
struct Args {
    pub input_file: PathBuf,
}

fn main() -> eyre::Result<()> {
    let args = Args::parse();
    ros_plan_parse::parse_plan_file(&args.input_file, None)?;
    Ok(())
}
