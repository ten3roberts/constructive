#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Span {
    pub min: f32,
    pub max: f32,
}

impl Span {
    pub fn new(min: f32, max: f32) -> Self {
        Self { min, max }
    }

    pub fn empty() -> Self {
        Self::new(0.0, 0.0)
    }

    pub fn is_empty(&self) -> bool {
        self.min >= self.max
    }

    pub fn intersect(&self, other: Self) -> Self {
        if self.is_empty() || other.is_empty() {
            Self::empty()
        } else {
            Self::new(self.min.max(other.min), self.max.min(other.max))
        }
    }
}
