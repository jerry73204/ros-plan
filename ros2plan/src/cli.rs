use clap::Parser;
use eyre::{bail, Context};
use ros_plan_format::{
    expr::{Value, ValueType},
    parameter::ParamName,
};
use std::{path::PathBuf, str::FromStr};

#[derive(Debug, Clone, Parser)]
pub enum Cli {
    Compile(CompileArgs),
}

// #[derive(Debug, Clone, Parser)]
// pub struct StartArgs {
//     /// The ROS plan file.
//     pub plan_file: PathBuf,

//     /// The execution environment configuration file.
//     #[clap(short = 'c', long)]
//     pub config: Option<PathBuf>,
// }

#[derive(Debug, Clone, Parser)]
pub struct CompileArgs {
    /// The ROS plan file.
    pub plan_file: PathBuf,

    /// The arguments passed to the program.
    pub args: Option<Vec<ArgAssign>>,

    /// The path to the output file.
    #[clap(short = 'o', long)]
    pub output_file: Option<PathBuf>,
}

#[derive(Debug, Clone)]
pub struct ArgAssign {
    pub name: ParamName,
    pub value: Value,
}

impl FromStr for ArgAssign {
    type Err = eyre::Error;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        let Some((name_with_ty, value_text)) = text.split_once("=") else {
            bail!("argument assignment must be writtien in `name=value` or `name:type=value`");
        };
        let (name, ty) = if let Some((name, ty)) = name_with_ty.split_once(":") {
            let ty: ValueType = ty.parse()?;
            (name, Some(ty))
        } else {
            (name_with_ty, None)
        };

        let name: ParamName = name
            .parse()
            .wrap_err_with(|| format!("invalid argument name `{name}`"))?;
        let yaml_value: serde_yaml::Value = serde_yaml::from_str(value_text)
            .wrap_err_with(|| format!("unable to parse value `{value_text}`"))?;
        let ros_value = Value::from_yaml_value(ty, yaml_value)
            .wrap_err_with(|| format!("unable to parse value `{value_text}`"))?;

        Ok(Self {
            name,
            value: ros_value,
        })
    }
}
