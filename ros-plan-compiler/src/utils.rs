pub mod arc_rwlock;
pub mod shared_table;
// pub mod shared_table2;

use crate::error::Error;
use serde::Deserialize;
use std::{fs, path::Path};

pub fn read_yaml_file<T, P>(path: P) -> Result<T, Error>
where
    T: for<'de> Deserialize<'de>,
    P: AsRef<Path>,
{
    let path = path.as_ref();
    let text = fs::read_to_string(path).map_err(|error| Error::ReadFileError {
        path: path.to_owned(),
        error,
    })?;
    let data = serde_yaml::from_str(&text).map_err(|error| Error::ParsePlanFileError {
        path: path.to_owned(),
        error,
    })?;
    Ok(data)
}
