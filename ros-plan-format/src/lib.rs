pub mod error;
pub mod expr;
pub mod ident;
pub mod key;
pub mod link;
pub mod node;
pub mod parameter;
pub mod plan;
pub mod qos;
pub mod qos_requirement;
pub mod ros_type;
pub mod socket;
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
