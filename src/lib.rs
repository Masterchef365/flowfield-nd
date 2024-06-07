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

impl<T: Copy> FatVector<T> {
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut T> + '_ {
        self.1.iter_mut()
    }

    pub fn iter(&self) -> impl Iterator<Item = &T> + '_ {
        self.1.iter()
    }

    fn map<U, F: FnMut(T) -> U>(self, f: F) -> FatVector<U> {
        let FatVector(size, elems) = self;
        FatVector(size, elems.map(f))
    }

    fn zip<U, V, F>(self, rhs: FatVector<U>, mut f: F) -> FatVector<V>
    where
        V: Default + Copy,
        F: FnMut(T, U) -> V,
    {
        let FatVector(size_lhs, elems_lhs) = self;
        let FatVector(size_rhs, elems_rhs) = rhs;
        assert_eq!(size_lhs, size_rhs);
        let mut output = FatVector::from_size(size_lhs);

        output
            .iter_mut()
            .zip(elems_lhs)
            .zip(elems_rhs)
            .for_each(|((output, lhs), rhs)| *output = f(lhs, rhs));

        output
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
