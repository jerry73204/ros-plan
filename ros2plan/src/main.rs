mod cli;
mod config;

use clap::Parser;
use cli::{ArgAssign, Cli, CompileArgs, EvalArgs, StartArgs};
use indexmap::IndexMap;
use ros_plan_compiler::{Compiler, Program};
use ros_plan_format::Plan;

fn main() -> eyre::Result<()> {
    let cli = Cli::parse();

    match cli {
        Cli::Start(args) => start(args)?,
        Cli::Compile(args) => compile(args)?,
        Cli::Eval(args) => eval(args)?,
    }

    Ok(())
}

fn start(args: StartArgs) -> eyre::Result<()> {
    todo!()
}

fn compile(args: CompileArgs) -> eyre::Result<()> {
    let compiler = Compiler::new();
    let program = compiler.compile(&args.plan_file)?;

    let text = program.to_string();
    if let Some(output_file) = args.output_file {
        std::fs::write(&output_file, text)?;
    } else {
        print!("{text}");
    }
    Ok(())
}

fn eval(args: EvalArgs) -> eyre::Result<()> {
    let EvalArgs {
        program_file,
        args,
        output_file,
    } = args;

    let mut program = Program::load(&program_file)?;

    let compiler = Compiler::new();
    let args: IndexMap<_, _> = match args {
        Some(args) => args
            .into_iter()
            .map(|ArgAssign { name, value }| (name, value))
            .collect(),
        None => IndexMap::new(),
    };
    compiler.eval(&mut program, args)?;

    let text = program.to_string();
    if let Some(output_file) = output_file {
        std::fs::write(&output_file, text)?;
    } else {
        print!("{text}");
    }

    Ok(())
}
