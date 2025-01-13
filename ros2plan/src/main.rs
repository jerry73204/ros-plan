mod cli;
mod config;

use clap::Parser;
use cli::{Cli, ExpandArgs, LoadArgs, StartArgs};
use indexmap::IndexMap;
use ros_plan_parse::Resource;

fn main() -> eyre::Result<()> {
    let cli = Cli::parse();

    match cli {
        Cli::Start(args) => start(args)?,
        Cli::Expand(args) => expand(args)?,
        Cli::Load(args) => load(args)?,
    }

    Ok(())
}

fn start(_args: StartArgs) -> eyre::Result<()> {
    todo!();
}

fn expand(args: ExpandArgs) -> eyre::Result<()> {
    let mut resource = Resource::from_plan_file(&args.plan_file)?;
    resource.eval(IndexMap::new())?;

    let text = serde_yaml::to_string(&resource)?;

    if let Some(output_file) = args.output_file {
        std::fs::write(&output_file, text)?;
    } else {
        print!("{text}");
    }
    Ok(())
}

fn load(args: LoadArgs) -> eyre::Result<()> {
    let mut resource: Resource = {
        let text = std::fs::read_to_string(&args.dump_file)?;
        serde_yaml::from_str(&text)?
    };

    resource.initialize_internal_references()?;

    {
        let text = serde_yaml::to_string(&resource)?;
        print!("{text}");
    }

    Ok(())
}
