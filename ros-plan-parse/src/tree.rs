use crate::{error::Error, utils::ArcRwLock};
use indexmap::IndexMap;
use ros_plan_format::key::{Key, KeyOwned};
use serde::Serialize;

pub type TreeRef<T> = ArcRwLock<Tree<T>>;

#[derive(Debug, Clone, Serialize)]
pub struct Tree<T> {
    pub value: T,
    pub children: IndexMap<KeyOwned, TreeRef<T>>,
}

impl<T> Tree<T> {
    pub fn new(value: T) -> Self {
        Self {
            value,
            children: IndexMap::new(),
        }
    }
}

impl<T> TreeRef<T> {
    pub fn new(value: T) -> Self {
        Tree::new(value).into()
    }

    pub fn get_child(&self, key: &Key) -> Option<Self> {
        let guard = self.read();
        let child = guard.children.get(key)?;
        Some(child.clone())
    }

    pub fn insert(&self, key: KeyOwned, value: T) -> Result<Self, Error> {
        // Reject absolute keys.
        if key.is_absolute() {
            return Err(Error::InvalidSubplanName {
                key: key.to_owned(),
                reason: "absolute key is not allowed".to_string(),
            });
        }

        // Reject empty keys.
        if key.is_empty() {
            return Err(Error::InvalidSubplanName {
                key: key.to_owned(),
                reason: "empty key not allowed".to_string(),
            });
        }

        // Try to insert the key
        let mut guard = self.write();
        let entry = match guard.children.entry(key.to_owned()) {
            indexmap::map::Entry::Vacant(entry) => entry,
            indexmap::map::Entry::Occupied(_) => {
                return Err(Error::ConflictingKeys {
                    old: key.to_owned(),
                    new: key.to_owned(),
                })
            }
        };

        // Create the child
        let child: TreeRef<T> = Tree {
            children: IndexMap::new(),
            value,
        }
        .into();
        entry.insert(child.clone());

        Ok(child)
    }
}
