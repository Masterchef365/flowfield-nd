use std::ops::{Index, IndexMut};

use ndarray::{Array, Array2, ArrayView, IxDyn};

#[derive(Clone)]
pub struct SolverConfig {
    dt: f32,
}

#[derive(Clone)]
pub struct FlowField {
    /// One single-component flow field for each face of the N-dimensional cube.
    flow: Vec<Array<f32, IxDyn>>,
}

pub struct FluidSolver {
    flow: FlowField,
}

/// This is a two-dimensional array for the point cloud, where the last dimension corresponds
/// to the coponent of the vector, and the second dimension corresponds to which point is selected
#[derive(Clone)]
pub struct PointCloud(Array2<f32>);

impl FluidSolver {
    pub fn new(flow: FlowField) -> Self {
        Self { flow }
    }

    pub fn step(&mut self, config: &SolverConfig) {
        todo!()
    }

    pub fn get_flow(&self) -> &FlowField {
        &self.flow
    }

    pub fn get_flow_mut(&mut self) -> &mut FlowField {
        &mut self.flow
    }

    /// Number of dimensions
    pub fn dims(&self) -> usize {
        self.flow.dims()
    }

    fn advect(&mut self, config: &SolverConfig) {}
}

/// Sweeps the given point cloud along the given flow field
pub fn sweep_pointcloud(pcld: &mut PointCloud, flow: &FlowField, dt: f32) {
    todo!()
}

impl FlowField {
    pub fn new(dimensions: usize, width: usize) -> Self {
        Self {
            flow: vec![Array::zeros(vec![width; dimensions]); dimensions],
        }
    }

    pub fn dims(&self) -> usize {
        self.flow.len()
    }

    /*
    /// Returns an iterator over the cell positions as an integer
    pub fn enumerate(&self) -> Box<dyn Iterator<Item = FatVector<usize>> + '_> {
        self.enumerate_rec(0, FatVector::from_size(self.dims()))
    }

    fn enumerate_rec(
        &self,
        idx: usize,
        fat: FatVector<usize>,
    ) -> Box<dyn Iterator<Item = FatVector<usize>> + '_> {
        let num_indices = self.0.shape().get(idx).copied().unwrap_or(0);
        let mut fat = fat;
        if num_indices == 0 {
            return Box::new(std::iter::once(fat))
        }
        Box::new(
            (0..num_indices)
                .map(move |j| {
                    fat[idx] = j;
                    self.enumerate_rec(idx + 1, fat)
                })
                .flatten(),
        )
    }
    */

    /// Bilinear interpolation at the given position in ND space
    pub fn bilinear(position: FatVector<f32>) -> Option<f32> {
    }
}

impl PointCloud {
    fn new(dimensions: usize) -> Self {
        Self(Array::zeros((dimensions, 0)))
    }
}
