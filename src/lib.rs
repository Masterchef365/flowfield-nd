use std::ops::{Index, IndexMut};

use ndarray::{Array, Array2, ArrayView, IxDyn};

#[derive(Clone)]
pub struct SolverConfig {
    dt: f32,
}

/// Total memory is of order O(width^(N+1))
/// Where N is the number of cell dimensions (3D -> N=3)
///
/// The last dimension of the Array corresponds to the component of the vector field.
/// So x corresponds to {N entries}, 0) and y corresponds to ({N entries}, 1) and so on up to ({N entries}, N)
///
/// note: {N entries} can be any rectangular N-dimensional shape,
/// but the last array dimension is always N
///
/// So the Array will have a shape with size (dimensions + 1)
#[derive(Clone)]
pub struct FlowField(Array<f32, IxDyn>);

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
    fn from_width(dimensions: usize, width: usize) -> Self {
        let mut arr_dimensions = vec![width; dimensions];
        // The last dimension implies a N-dimensional vector for each Cell.
        arr_dimensions.push(dimensions);

        Self(Array::zeros(vec![width; dimensions + 1]))
    }

    fn dims(&self) -> usize {
        self.0.ndim() - 1
    }

    /// Returns an iterator over the cell positions as an integer
    fn enumerate(&self) -> Box<dyn Iterator<Item = FatVector<usize>> + '_> {
        self.enumerate_rec(0, FatVector::from_size(self.dims()))
    }

    fn enumerate_rec(
        &self,
        idx: usize,
        fat: FatVector<usize>,
    ) -> Box<dyn Iterator<Item = FatVector<usize>> + '_> {
        let num_indices = self.0.shape().get(idx).copied().unwrap_or(0);
        let mut fat = fat;
        Box::new(
            (0..num_indices)
                .map(move |j| {
                    fat[idx] = j;
                    self.enumerate_rec(idx + 1, fat)
                })
                .flatten(),
        )
    }

    /// Bilinear interpolation at the given position in ND space
    fn bilinear(position: FatVector<f32>) -> f32 {
        todo!()
    }
}

impl PointCloud {
    fn new(dimensions: usize) -> Self {
        Self(Array::zeros((dimensions, 0)))
    }
}

/// Assume we aren't using more than this many dimensions
const FAT: usize = 24;

/// The typical (size, array) tuple setup
#[derive(Clone, Copy, Debug)]
pub struct FatVector<T>(usize, [T; FAT]);

impl<T: Default + Copy> FatVector<T> {
    fn from_size(size: usize) -> Self {
        FatVector(size, [T::default(); FAT])
    }
}

impl<T> Index<usize> for FatVector<T> {
    type Output = T;
    fn index(&self, index: usize) -> &Self::Output {
        &self.1[index]
    }
}

impl<T> IndexMut<usize> for FatVector<T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.1[index]
    }
}
