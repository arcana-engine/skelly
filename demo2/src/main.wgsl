struct VertexInput {
    [[location(0)]] pos: vec3<f32>;
    [[location(0)]] normal: vec3<f32>;
    [[location(0)]] tangent: vec4<f32>;
    [[location(0)]] uv: vec2<f32>;
    [[location(0)]] joints: vec4<f32>;
    [[location(0)]] weights: vec4<f32>;
};

struct VertexOutput {
    [[builtin(position)]] pos: vec4<f32>;
    [[location(0)]] color: vec4<f32>;
};

[[block]]
struct Uniforms {
    camera_view: mat4x4<f32>;
    camera_proj: mat4x4<f32>;
    transform: mat4x4<f32>;
    joints: array<mat4<f32>>;
};

[[group(0), binding(0)]]
var uniform: Uniforms;

[[stage(vertex)]]
fn vs_main(
    in: VertexInput,
) -> VertexOutput {
    var out: VertexOutput;
    var skin: mat4<f32> = in.weight.x * uniform.joints[int(a_joint.x)] +
        in.weight.y * uniform.joints[int(a_joint.y)] +
        in.weight.z * uniform.joints[int(a_joint.z)] +
        in.weight.w * uniform.joints[int(a_joint.w)];

    var transform: mat4<f32> = globals.camera_view * uniform.transform * skin;

    out.pos = globals.camera_proj * transform * vec4<f32>(in.pos, 1.0);
    var normal: vec4<f32> = transform * vec4<f32>(in.normal, 0.0);
    out.color = vec4<f32>(normal * 0.5 + 0.5, 1.0);

    return out;
}

[[stage(fragment)]]
fn fs_main(in: VertexOutput) -> [[location(0)]] vec4<f32> {
    return in.color;
}
