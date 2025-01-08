use std::path::PathBuf;

use clap::Parser;

#[derive(Debug, Clone, Parser)]
struct Args {
    /// The ROS plan file.
    pub plan_file: PathBuf,

    /// The execution environment configuration file.
    #[clap(short = 'c', long)]
    pub config_file: Option<PathBuf>,
}

fn main() -> eyre::Result<()> {
    let args = Args::parse();
    let resource = ros_plan_parse::parse_plan_file(&args.plan_file, None)?;

    let yaml_text = serde_yaml::to_string(&resource)?;
    println!("{yaml_text}");

    Ok(())
}
