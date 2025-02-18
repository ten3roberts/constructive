pub mod astar;
pub mod brush;
pub mod edge;
pub mod edgelist;
pub mod link;
pub mod navmesh;
pub mod plane;
pub mod span;
pub mod tree;
mod util;

pub fn add(left: u64, right: u64) -> u64 {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
