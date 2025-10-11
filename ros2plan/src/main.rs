mod cli;
mod config;

use clap::Parser;
use cli::{ArgAssign, Cli, CompileArgs, RunArgs};
use indexmap::IndexMap;
use ros_plan_compiler::Compiler;
use ros_plan_runtime::Runtime;

fn main() -> eyre::Result<()> {
    let cli = Cli::parse();

    match cli {
        Cli::Compile(args) => compile(args)?,
        Cli::Run(args) => run(args)?,
    }

    Ok(())
}

fn compile(opts: CompileArgs) -> eyre::Result<()> {
    let CompileArgs {
        plan_file,
        args,
        output_file,
    } = opts;

    let args: IndexMap<_, _> = match args {
        Some(args) => args
            .into_iter()
            .map(|ArgAssign { name, value }| (name, value))
            .collect(),
        None => IndexMap::new(),
    };

    let compiler = Compiler::new();
    let program = compiler.compile(&plan_file, args)?;

    let text = program.to_string();
    if let Some(output_file) = output_file {
        std::fs::write(&output_file, text)?;
    } else {
        print!("{text}");
    }
    Ok(())
}

fn run(opts: RunArgs) -> eyre::Result<()> {
    let RunArgs { plan_file, args } = opts;

    let args: IndexMap<_, _> = match args {
        Some(args) => args
            .into_iter()
            .map(|ArgAssign { name, value }| (name, value))
            .collect(),
        None => IndexMap::new(),
    };

    let mut runtime = Runtime::new(plan_file, args)?;
    runtime.run()?;

    Ok(())
}
