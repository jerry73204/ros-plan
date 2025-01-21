use crate::{
    error::IdentifierCreationError,
    key::{Key, KeyOwned},
};
use pomsky_macro::pomsky;
use regex::Regex;
use serde::{Deserialize, Serialize};
use std::{
    borrow::Borrow,
    fmt::{self, Debug, Display},
    ops::Deref,
    str::FromStr,
    sync::LazyLock,
};

pub static RE_IDENT: LazyLock<Regex> = LazyLock::new(|| {
    let rule = pomsky! {
        ^ ['a'-'z' 'A'-'Z' '_'] [ascii_word]* $
    };
    Regex::new(rule).expect("invalid regex for identifier")
});

#[derive(PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Ident(pub(crate) str);

impl Ident {
    pub fn to_key(&self) -> KeyOwned {
        KeyOwned(self.0.to_string())
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    pub fn as_key(&self) -> &Key {
        self.into()
    }
}

impl Debug for Ident {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        Debug::fmt(&self.0, f)
    }
}

impl Display for Ident {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        Display::fmt(&self.0, f)
    }
}

impl ToOwned for Ident {
    type Owned = IdentOwned;

    fn to_owned(&self) -> Self::Owned {
        IdentOwned(self.0.to_string())
    }
}

impl FromStr for &Ident {
    type Err = IdentifierCreationError;

    fn from_str(name: &str) -> Result<Self, Self::Err> {
        if RE_IDENT.is_match(name) {
            let ident: &Ident = unsafe { std::mem::transmute(name) };
            Ok(ident)
        } else {
            Err(IdentifierCreationError {
                string: name.to_string(),
            })
        }
    }
}

impl TryFrom<&Key> for &Ident {
    type Error = IdentifierCreationError;

    fn try_from(key: &Key) -> Result<Self, Self::Error> {
        if !key.is_ident() {
            return Err(IdentifierCreationError {
                string: key.to_string(),
            });
        }
        let ident: &Ident = unsafe { std::mem::transmute(key) };
        Ok(ident)
    }
}

#[derive(Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(try_from = "String", into = "String")]
pub struct IdentOwned(pub(crate) String);

impl Deref for IdentOwned {
    type Target = Ident;

    fn deref(&self) -> &Self::Target {
        unsafe { std::mem::transmute(self.0.as_str()) }
    }
}

impl AsRef<Ident> for IdentOwned {
    fn as_ref(&self) -> &Ident {
        unsafe { std::mem::transmute(self.0.as_str()) }
    }
}

impl Borrow<Ident> for IdentOwned {
    fn borrow(&self) -> &Ident {
        unsafe { std::mem::transmute(self.0.as_str()) }
    }
}

impl TryFrom<String> for IdentOwned {
    type Error = IdentifierCreationError;

    fn try_from(name: String) -> Result<Self, Self::Error> {
        name.parse()
    }
}

impl FromStr for IdentOwned {
    type Err = IdentifierCreationError;

    fn from_str(name: &str) -> Result<Self, Self::Err> {
        let name = name.to_string();

        if RE_IDENT.is_match(&name) {
            Ok(Self(name))
        } else {
            Err(IdentifierCreationError { string: name })
        }
    }
}

impl From<IdentOwned> for String {
    fn from(ident: IdentOwned) -> Self {
        ident.0.to_string()
    }
}

impl Debug for IdentOwned {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        Debug::fmt(self.as_ref(), f)
    }
}

impl Display for IdentOwned {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        Display::fmt(self.as_ref(), f)
    }
}
