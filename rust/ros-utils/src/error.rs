#[derive(Debug, Clone, thiserror::Error)]
#[error("unable to resolve directory for package `{pkg}`: {message}")]
pub struct PackageResolutionError {
    pub pkg: String,
    pub message: String,
}

#[derive(Debug, Clone, thiserror::Error)]
#[error("fail to parse node command line `{cmdline:?}`: {reason}")]
pub struct ParseNodeCmdlineError {
    pub cmdline: Vec<String>,
    pub reason: String,
}
