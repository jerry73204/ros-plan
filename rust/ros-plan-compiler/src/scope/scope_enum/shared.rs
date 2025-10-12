use super::{ScopeOwned, ScopeReadGuard, ScopeWriteGuard};
use crate::scope::{GroupScopeShared, PlanScopeShared};

#[derive(Debug, Clone)]
pub enum ScopeShared {
    Group(GroupScopeShared),
    Include(PlanScopeShared),
}

impl ScopeShared {
    pub fn id(&self) -> usize {
        match self {
            ScopeShared::Group(shared) => shared.id(),
            ScopeShared::Include(shared) => shared.id(),
        }
    }

    pub fn upgrade(&self) -> Option<ScopeOwned> {
        Some(match self {
            ScopeShared::Group(shared) => shared.upgrade()?.into(),
            ScopeShared::Include(shared) => shared.upgrade()?.into(),
        })
    }

    pub fn with_read<R, F>(&self, f: F) -> R
    where
        F: FnOnce(ScopeReadGuard) -> R,
    {
        let owned = self.upgrade().unwrap();
        let guard = owned.read();
        f(guard)
    }

    pub fn with_write<R, F>(&self, f: F) -> R
    where
        F: FnOnce(ScopeWriteGuard) -> R,
    {
        let owned = self.upgrade().unwrap();
        let guard = owned.write();
        f(guard)
    }

    pub fn as_group(&self) -> Option<&GroupScopeShared> {
        if let Self::Group(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_group_mut(&mut self) -> Option<&mut GroupScopeShared> {
        if let Self::Group(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_include(&self) -> Option<&PlanScopeShared> {
        if let Self::Include(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_include_mut(&mut self) -> Option<&mut PlanScopeShared> {
        if let Self::Include(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn try_into_group(self) -> Result<GroupScopeShared, Self> {
        if let Self::Group(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_include(self) -> Result<PlanScopeShared, Self> {
        if let Self::Include(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    #[must_use]
    pub fn ptr_eq(&self, other: &Self) -> bool {
        match (self, other) {
            (ScopeShared::Group(lhs), ScopeShared::Group(rhs)) => lhs.ptr_eq(rhs),
            (ScopeShared::Include(lhs), ScopeShared::Include(rhs)) => lhs.ptr_eq(rhs),
            _ => false,
        }
    }
}

impl From<PlanScopeShared> for ScopeShared {
    fn from(v: PlanScopeShared) -> Self {
        Self::Include(v)
    }
}

impl From<GroupScopeShared> for ScopeShared {
    fn from(v: GroupScopeShared) -> Self {
        Self::Group(v)
    }
}
