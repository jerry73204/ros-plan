pub mod argument;
pub mod error;
pub mod expr;
pub mod ident;
pub mod interface_type;
pub mod key;
pub mod link;
pub mod node;
pub mod node_socket;
pub mod parameter;
pub mod plan;
pub mod plan_socket;
pub mod qos;
pub mod qos_requirement;
pub mod subplan;

pub use crate::plan::Plan;

#[cfg(test)]
mod tests {
    use crate::Plan;
    use std::fs;

    #[test]
    fn parse_example() {
        let text = fs::read_to_string("example.toml").expect("unable to read example.toml");
        let _plan: Plan = toml::from_str(&text).expect("unable to parse example.toml");
    }
}
