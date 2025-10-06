use crate::error::ParseNodeCmdlineError;
use itertools::{chain, Itertools};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize, Serializer};
use std::{
    borrow::{Borrow, Cow},
    collections::{HashMap, HashSet},
    path::PathBuf,
    process::Command,
};

/// The command line information to execute a ROS node.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NodeCommandLine {
    pub command: Vec<String>,
    pub user_args: Vec<String>,
    pub remaps: HashMap<String, String>,
    pub params: HashMap<String, String>,
    pub params_files: HashSet<PathBuf>,
    pub log_level: Option<String>,
    pub log_config_file: Option<PathBuf>,
    pub rosout_logs: Option<bool>,
    pub stdout_logs: Option<bool>,
    pub enclave: Option<String>,
}

impl NodeCommandLine {
    /// Construct from command line arguments.
    pub fn from_cmdline<A>(cmdline: &[A]) -> Result<Self, ParseNodeCmdlineError>
    where
        A: AsRef<str>,
    {
        macro_rules! bail {
            ($($msg:tt)*) => {
                {
                    return Err(ParseNodeCmdlineError {
                        cmdline: cmdline.iter().map(|arg| arg.as_ref().to_string()).collect(),
                        reason: format!($($msg)*),
                    });
                }
            };
        }

        let (command, user_args, ros_args) = {
            let mut iter = cmdline.iter();
            let Some(command) = iter.next() else {
                bail!("the command line must not be empty");
            };

            let mut user_args = vec![];
            let mut ros_args = vec![];

            'arg_loop: loop {
                loop {
                    let Some(arg) = iter.next() else {
                        break 'arg_loop;
                    };

                    match arg.as_ref() {
                        "--ros-args" => break,
                        arg => {
                            user_args.push(arg.to_string());
                        }
                    }
                }

                loop {
                    let Some(arg) = iter.next() else {
                        break 'arg_loop;
                    };

                    match arg.as_ref() {
                        "--ros-args" => continue,
                        "--" => break,
                        _ => {
                            ros_args.push(arg);
                        }
                    }
                }
            }

            (command.as_ref().to_string(), user_args, ros_args)
        };

        let mut iter = ros_args.into_iter();
        let mut remaps = HashMap::new();
        let mut params = HashMap::new();
        let mut params_files = HashSet::new();
        let mut log_level = None;
        let mut log_config_file = None;
        let mut rosout_logs = None;
        let mut stdout_logs = None;
        let mut enclave = None;

        loop {
            let Some(arg) = iter.next() else { break };

            match arg.as_ref() {
                "-r" | "--remap" => {
                    let Some(arg2) = iter.next() else {
                        bail!("expect an argument after -r/--remap");
                    };
                    let Some((name, value)) = arg2.as_ref().split_once(":=") else {
                        bail!("invalid assignment {}", arg2.as_ref())
                    };
                    remaps.insert(name.to_string(), value.to_string());
                }
                "-p" | "--param" => {
                    let Some(arg2) = iter.next() else {
                        bail!("expect an argument after -p/--param");
                    };
                    let Some((name, value)) = arg2.as_ref().split_once(":=") else {
                        bail!("invalid assignment {}", arg2.as_ref())
                    };
                    params.insert(name.to_string(), value.to_string());
                }
                "--params-file" => {
                    let Some(arg2) = iter.next() else {
                        bail!("expect an argument after -params-file");
                    };
                    params_files.insert(PathBuf::from(arg2.as_ref()));
                }
                "--log-level" => {
                    let Some(level) = iter.next() else {
                        bail!("expect an argument after --log-level");
                    };
                    log_level = Some(level.as_ref().to_string());
                }
                "--log-config-file" => {
                    let Some(path) = iter.next() else {
                        bail!("expect an argument after --log-config-file");
                    };
                    log_config_file = Some(PathBuf::from(path.as_ref()));
                }
                "--enable-rosout-logs" => {
                    rosout_logs = Some(true);
                }
                "--disable-rosout-logs" => {
                    rosout_logs = Some(false);
                }
                "--enable-stdout-logs" => {
                    stdout_logs = Some(false);
                }
                "--disable-stdout-logs" => {
                    stdout_logs = Some(false);
                }
                "-e" | "--enclave" => {
                    let Some(arg2) = iter.next() else {
                        bail!("expect an argument after -e/--enclave");
                    };
                    enclave = Some(arg2.as_ref().to_string())
                }
                arg => {
                    bail!("unexpected argument {arg} after --ros-arg");
                }
            }
        }

        Ok(Self {
            command: vec![command],
            user_args,
            remaps,
            params,
            params_files,
            log_level,
            log_config_file,
            rosout_logs,
            stdout_logs,
            enclave,
        })
    }

    /// Create command line arguments.
    pub fn to_cmdline(&self, long_args: bool) -> Vec<String> {
        let Self {
            command,
            user_args,
            remaps,
            params,
            params_files,
            log_level,
            log_config_file,
            rosout_logs,
            stdout_logs,
            enclave,
        } = self;

        let has_ros_args = !(remaps.is_empty()
            && params.is_empty()
            && params_files.is_empty()
            && log_level.is_none()
            && log_config_file.is_none()
            && rosout_logs.is_none()
            && stdout_logs.is_none()
            && enclave.is_none());

        let ros_args: Vec<_> = has_ros_args
            .then(|| {
                let param_switch = if long_args { "--param" } else { "-p" };
                let remap_switch = if long_args { "--remap" } else { "-r" };

                let log_level_args = log_level
                    .as_ref()
                    .map(|level| ["--log-level", level])
                    .into_iter()
                    .flatten()
                    .map(Cow::from);
                let log_config_file_args = log_config_file
                    .as_ref()
                    .map(|path| ["--log-config-file", path.to_str().unwrap()])
                    .into_iter()
                    .flatten()
                    .map(Cow::from);
                let rosout_logs_args = rosout_logs
                    .map(|yes| {
                        if yes {
                            "--enable-rosout-logs"
                        } else {
                            "--disable-rosout-logs"
                        }
                    })
                    .map(Cow::from);
                let stdout_logs_args = stdout_logs
                    .map(|yes| {
                        if yes {
                            "--enable-stdout-logs"
                        } else {
                            "--disable-stdout-logs"
                        }
                    })
                    .map(Cow::from);
                let enclave_args = enclave
                    .as_ref()
                    .map(|value| ["--enclave", value])
                    .into_iter()
                    .flatten()
                    .map(Cow::from);
                let remap_args = remaps.iter().flat_map(move |(name, value)| {
                    [Cow::from(remap_switch), format!("{name}:={value}").into()]
                });
                let params_args = params.iter().flat_map(move |(name, value)| {
                    [Cow::from(param_switch), format!("{name}:={value}").into()]
                });
                let params_file_args = params_files
                    .iter()
                    .flat_map(|path| ["--params-file", path.to_str().unwrap()])
                    .map(Cow::from);

                chain!(
                    [Cow::from("--ros-args")],
                    log_level_args,
                    log_config_file_args,
                    rosout_logs_args,
                    stdout_logs_args,
                    enclave_args,
                    remap_args,
                    params_args,
                    params_file_args,
                )
            })
            .into_iter()
            .flatten()
            .collect();

        let words: Vec<_> = chain!(
            command.iter().map(|arg| arg.as_str()),
            user_args.iter().map(|arg| arg.as_str()),
            ros_args.iter().map(|arg| arg.borrow()),
        )
        .map(|arg| arg.to_string())
        .collect();

        words
    }

    /// Create a command object.
    pub fn to_command(&self, long_args: bool) -> Command {
        let cmdline = self.to_cmdline(long_args);
        let (program, args) = cmdline.split_first().unwrap();
        let mut command = Command::new(program);
        command.args(args);
        command
    }

    /// Generate the shell script.
    pub fn to_shell(&self, long_args: bool) -> Vec<u8> {
        Itertools::intersperse(
            self.to_cmdline(long_args)
                .into_iter()
                .map(|arg| shell_quote::Sh::quote_vec(&arg)),
            vec![b' '],
        )
        .flatten()
        .collect()
    }
}

impl Serialize for NodeCommandLine {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        self.to_cmdline(true).serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for NodeCommandLine {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let cmdline = Vec::<String>::deserialize(deserializer)?;
        let cmd = Self::from_cmdline(&cmdline).map_err(|err| D::Error::custom(format!("{err}")))?;
        Ok(cmd)
    }
}
