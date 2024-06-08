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
    width: usize,
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

    pub fn width(&self) -> usize {
        self.flow.width()
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
            width,
        }
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn dims(&self) -> usize {
        self.flow.len()
    }

    /// Returns an iterator over the cell positions (as integers)
    pub fn enumerate(&self) -> impl Iterator<Item = Vec<usize>> + 'static {
        enumerate_coords(self.width(), self.dims())
    }

    /// Bilinear interpolation at the given position in ND space
    pub fn bilinear(position: &[f32]) -> Option<Vec<f32>> {
        todo!()
    }
}

impl PointCloud {
    fn new(dimensions: usize) -> Self {
        Self(Array::zeros((dimensions, 0)))
    }
}

pub fn enumerate_coords(width: usize, n_dims: usize) -> impl Iterator<Item = Vec<usize>> + 'static {
    let mut coordinate = vec![0; n_dims];
    std::iter::from_fn(move || {
        let inc = coordinate.iter().position(|c| c + 1 != width)?;
        let cpy = coordinate.clone();
        coordinate[inc] += 1;
        coordinate[0..inc].fill(0);
        Some(cpy)
    }).chain(std::iter::once(vec![width - 1; n_dims]))
}

pub fn neighborhood(n_dims: usize) -> Vec<Vec<i32>> {
    combos(-1, 1, 1, n_dims)
}

fn combos(min: i32, max: i32, step: i32, n_dims: usize) -> Vec<Vec<i32>> {
    let mut dims = vec![min; n_dims];
    let mut combos = vec![];
    loop {
        combos.push(dims.clone());
        if dims == vec![max; n_dims] {
            break combos;
        }
        for i in 0..n_dims {
            if dims[i] < max {
                dims[i] += step;
                break;
            } else {
                dims[i] = min;
            }
        }
    }
}
