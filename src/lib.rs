use ndarray::{Array, Array2, IxDyn};

#[derive(Clone)]
pub struct SolverConfig {
    dt: f32,
}

/// Total memory is of order O(width^dimensions)
///
/// The first dimension of the Array corresponds to the component of the vector field.
/// So x corresponds to (0, ... ) and y corresponds to (1, ... ) and so on.
/// The remaining dimensions are the actual interpolation grid.
/// So the Array will have a shape with size (dimensions + 1) 
#[derive(Clone)]
pub struct FlowField(Array<f32, IxDyn>);

pub struct FluidSolver {
    flow: FlowField,
}

/// This is a two-dimensional array for the point cloud, where the first dimension corresponds
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

    pub fn flow(&self) -> &FlowField {
        &self.flow
    }

    pub fn flow_mut(&mut self) -> &mut FlowField {
        &mut self.flow
    }
}

/// Sweeps the given point cloud along the given flow field
pub fn sweep_pointcloud(pcld: &mut PointCloud, flow: &FlowField, dt: f32) {
    todo!()
}

impl FlowField {
    fn new(dimensions: usize, width: usize) -> Self {
        Self(Array::zeros(vec![width; dimensions + 1]))
    }
}

impl PointCloud {
    fn new(dimensions: usize) -> Self {
        Self(Array::zeros((dimensions, 0)))
    }
}

