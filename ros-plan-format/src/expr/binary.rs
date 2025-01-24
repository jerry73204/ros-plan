use base64::prelude::*;
use serde::{de::Error as _, Deserialize, Deserializer, Serialize, Serializer};

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct Base64String(pub Vec<u8>);

impl Base64String {
    pub fn as_slice(&self) -> &[u8] {
        &self.0
    }
}

impl From<Vec<u8>> for Base64String {
    fn from(value: Vec<u8>) -> Self {
        Self(value)
    }
}

impl From<Base64String> for Vec<u8> {
    fn from(value: Base64String) -> Self {
        value.0
    }
}

impl Serialize for Base64String {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        BASE64_STANDARD.encode(&self.0).serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Base64String {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let base64_text = String::deserialize(deserializer)?;
        let binary = BASE64_STANDARD
            .decode(&base64_text)
            .map_err(|err| D::Error::custom(format!("{err}")))?;
        Ok(Self(binary))
    }
}
