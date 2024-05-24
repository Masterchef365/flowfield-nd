use ndarray::{Array, Array2, IxDyn};

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
}

impl PointCloud {
    fn new(dimensions: usize) -> Self {
        Self(Array::zeros((dimensions, 0)))
    }
}


