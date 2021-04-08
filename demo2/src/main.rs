use {
    bumpalo::{collections::Vec as BVec, Bump},
    sierra::*,
    std::convert::TryFrom as _,
    tracing_subscriber::layer::SubscriberExt as _,
};

fn main() -> eyre::Result<()> {
    color_eyre::install()?;

    tracing::subscriber::set_global_default(
        tracing_subscriber::fmt()
            .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
            .pretty()
            .finish()
            .with(tracing_error::ErrorLayer::default()),
    )?;

    let event_loop = winit::event_loop::EventLoop::new();
    let window = winit::window::Window::new(&event_loop)?;

    let graphics = Graphics::get_or_init()?;
    let mut surface = graphics.create_surface(&window)?;

    let physical = graphics
        .devices()?
        .into_iter()
        .max_by_key(|d| d.info().kind)
        .ok_or_else(|| eyre::eyre!("Failed to find physical device"))?;

    let (device, queue) = physical.create_device(&[], SingleQueueQuery::GRAPHICS)?;

    let shader_module = device.create_shader_module(sierra::ShaderModuleInfo {
        code: std::fs::read(
            std::path::Path::new(std::env!("CARGO_MANIFEST_DIR")).join("main.wgsl"),
        )?
        .into(),
        language: sierra::ShaderLanguage::WGSL,
    })?;

    let mut swapchain = device.create_swapchain(&mut surface)?;
    swapchain.configure(
        ImageUsage::COLOR_ATTACHMENT,
        Format::BGRA8Snorm,
        PresentMode::Fifo,
    )?;

    let pipeline_layout = Pipeline::layout(&device)?;

    let renderer = Demo {
        swapchain,

        pawn: Buffer,

        descriptors_instance: pipeline_layout.set.instance(),
        pipeline_layout,

        pawn_pipeline: DynamicGraphicsPipeline::new(sierra::graphics_pipeline_desc! {
            vertex_shader: sierra::VertexShader::new(shader_module.clone(), "vs_main"),
            layout: pipeline_layout.raw().clone(),
        }),

        main: Main::instance(),

        fences: [
            device.create_fence()?,
            device.create_fence()?,
            device.create_fence()?,
        ],
        fence_index: 0,
        device,
        queue,
    };

    Ok(())
}

#[shader_repr]
struct Camera {
    view: mat4,
    proj: mat4,
}

impl Camera {
    fn new(aspect: f32) -> Self {
        let proj = na::geometry::Perspective3::new(aspect, std::f32::consts::FRAC_PI_2, 0.1, 100.0);

        let proj = proj.to_homogeneous();
        let proj = mat4::try_from(proj.as_slice()).unwrap();

        let view = na::Matrix4::face_towards(
            &(na::Point3::origin() - na::Vector3::x() * 10.0),
            &na::Point3::origin(),
            &na::Vector3::y(),
        );
        let view = mat4::try_from(view.as_slice()).unwrap();
        Camera { view, proj }
    }
}

#[shader_repr]
struct Globals {
    camera: Camera,
    instant: f32,
}

#[shader_repr]
struct Object {
    transform: mat4,
    joints: [mat4; 128],
}

#[descriptors]
struct Descriptors {
    #[uniform]
    #[stages(Vertex)]
    globals: Globals,

    #[uniform]
    #[stages(Vertex)]
    object: Object,
}

#[pipeline]
struct Pipeline {
    #[set]
    set: Descriptors,
}

const LIGHTSTEELBLUE: palette::Srgb<u8> = palette::named::LIGHTSTEELBLUE;

const BACKGROUND: ClearColor = ClearColor(
    LIGHTSTEELBLUE.red as f32,
    LIGHTSTEELBLUE.green as f32,
    LIGHTSTEELBLUE.blue as f32,
    1.0,
);

#[pass]
struct Main {
    #[attachment(clear(const BACKGROUND), store(const Layout::Present))]
    target: Image,

    #[attachment(clear(const ClearDepth(1.0)))]
    depth: Format,
}

struct Demo {
    device: Device,
    queue: Queue,
    swapchain: Swapchain,

    pawn: Buffer,

    pipeline_layout: PipelineLayout,
    descriptors_instance: DescriptorsInstance,
    descriptors: Descriptors,

    pawn_pipeline: DynamicGraphicsPipeline,

    main: MainInstance,

    fences: [Fence; 3],
    fence_index: usize,
}

impl Demo {
    fn draw(&mut self, bump: &Bump) -> eyre::Result<()> {
        let image = self.swapchain.acquire_image(false)?;

        let mut encoder = self.queue.create_encoder(bump)?;
        let mut render_encoder = self.queue.create_encoder(bump)?;
        let mut render_pass = render_encoder.with_render_pass(
            &mut self.main,
            &Main {
                target: image.info().image.clone(),
                depth: Format::D32Sfloat,
            },
            &self.device,
        )?;
        let extent = render_pass.framebuffer().info().extent;

        let mut writes = BVec::new_in(bump);
        let updated = self.set.update(
            &Descriptors {
                globals: Globals {
                    camera: Camera::new(extent.width as f32 / extent.height as f32),
                    instant: 0.0,
                },
            },
            self.fence_index,
            &self.device,
            &mut writes,
            &mut encoder,
        )?;

        render_pass.bind_graphics_descriptors(&self.pipeline_layout, updated);
        render_pass.bind_vertex_buffers(0, &[(&self.pawn, 0)]);

        Ok(())
    }
}
