type SizeNd = ();


#[derive(Clone)]
pub struct SolverConfig {
    dt: f32,
}

#[derive(Clone)]
pub struct FlowField {
}

pub struct FluidSolver {
    cfg: SolverConfig,
    flow: FlowField,
}

#[derive(Clone)]
pub struct PointCloud {
}

impl FluidSolver {
    pub fn new(size: SizeNd) -> Self {
        todo!()
    }

    pub fn with_config(self, flow: SolverConfig) -> Self {

    }

    pub fn with_flowfield(self, flow: FlowField) -> Self {
        todo!()
    }

    pub fn step(&mut self) {
        todo!()
    }
}

/// Sweeps the given point cloud along the given flow field
pub fn sweep_pointcloud(pcld: &mut PointCloud, flow: &FlowField, dt: f32) {
    todo!()
}
