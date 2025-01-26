mod cli;
mod config;

use clap::Parser;
use cli::{Cli, ExpandArgs, LoadArgs, StartArgs};
use indexmap::IndexMap;
use ros_plan_compiler::Program;
use ros_plan_format::Plan;

fn main() -> eyre::Result<()> {
    let cli = Cli::parse();

    match cli {
        Cli::Start(args) => start(args)?,
        Cli::Expand(args) => expand(args)?,
        Cli::Load(args) => load(args)?,
    }

    Ok(())
}

fn start(args: StartArgs) -> eyre::Result<()> {
    let toml_text = std::fs::read_to_string(&args.plan_file)?;
    let plan: Plan = toml::from_str(&toml_text)?;
    let yaml_text = serde_yaml::to_string(&plan)?;
    print!("{yaml_text}");
    Ok(())
}

fn expand(args: ExpandArgs) -> eyre::Result<()> {
    let mut resource = Program::from_plan_file(&args.plan_file)?;
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
    let mut resource: Program = {
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
