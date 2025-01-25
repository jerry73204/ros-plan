mod entity_enum;
mod group;
mod plan_file;
mod scope_enum;
mod traits;

pub use entity_enum::{EntityOwned, EntityShared};
pub use group::GroupScope;
pub use plan_file::PlanFileScope;
pub use scope_enum::{ScopeOwned, ScopeShared};
pub use traits::{KeyKind, ScopeEntry, ScopeMut, ScopeMutExt, ScopeRef, ScopeRefExt};

use crate::{
    context::{
        link::LinkContext, node::NodeContext, node_socket::NodeSocketContext,
        plan_socket::PlanSocketContext,
    },
    utils::shared_table::{Owned, Shared},
};

pub type NodeOwned = Owned<NodeContext>;
pub type NodeShared = Shared<NodeContext>;

pub type LinkOwned = Owned<LinkContext>;
pub type LinkShared = Shared<LinkContext>;

pub type PlanSocketOwned = Owned<PlanSocketContext>;
pub type PlanSocketShared = Shared<PlanSocketContext>;

pub type NodeSocketOwned = Owned<NodeSocketContext>;
pub type NodeSocketShared = Shared<NodeSocketContext>;

pub type PlanFileScopeOwned = Owned<PlanFileScope>;
pub type PlanFileScopeShared = Shared<PlanFileScope>;

pub type GroupScopeOwned = Owned<GroupScope>;
pub type GroupScopeShared = Shared<GroupScope>;
