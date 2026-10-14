function test_section2_basics()
% Run: matlab -batch "test_section2_basics"
global dt e3 m_L J_L Jx_L Jy_L Jz_L
load_lifting_parameters;
e3 = [0;0;1];
cfg = load_lifting_controller_params('rbf', false);
base_cfg = load_lifting_controller_params('backstepping', false);
cfg.payload_control.backstepping = base_cfg.payload_control.backstepping;
cfg.rbf.base_mode = 'backstepping';
cfg.rbf.enable_compensation = false; cfg.rbf.enable_adaptation = false;
cfg.eso.use_compensation = false;
% Hover and units.
x = zeros(12,1);
d = load_lifting_payload_dynamics(x, [0;0;m_L*9.81], zeros(3,1), 0, false);
assert(norm(d) < 1e-12);
% Nonzero attitude: Rdot = R*skew(omega), analytic Tdot checked numerically.
eta = [0.3;-0.4;0.7]; omega = [0.2;-0.1;0.4];
[T,Tdot,edot] = zyx_euler_kinematics(eta,omega);
h = 1e-6; R = eul2rotm_ZYX(eta);
Rdot = (eul2rotm_ZYX(eta+h*edot)-eul2rotm_ZYX(eta-h*edot))/(2*h);
assert(norm(Rdot-R*skew(omega),'fro') < 1e-8);
Tp = zyx_euler_kinematics(eta+h*edot,omega);
Tm = zyx_euler_kinematics(eta-h*edot,omega);
assert(norm((Tp-Tm)/(2*h)-Tdot,'fro') < 1e-8);
assert(norm(rotm2eul_ZYX(R)-eta) < 1e-12);
% Analytic reference derivatives, proper rotations, no startup zeros.
t = 0:0.001:20;
[P,Rd,A] = load_lifting_desireTrajectory(t,0.001);
assert(abs(A(3,2,1)) > 0.1);
for k = [1,3000,10000,20001]
    assert(norm(Rd(:,:,k)'*Rd(:,:,k)-eye(3),'fro') < 1e-12);
    assert(abs(det(Rd(:,:,k))-1) < 1e-12);
    [pp,~,ap] = load_lifting_desireTrajectory(t(k)+h,h);
    [pm,~,am] = load_lifting_desireTrajectory(t(k)-h,h);
    assert(norm((pp(:,1)-pm(:,1))/(2*h)-P(:,2,k)) < 1e-8);
    assert(norm((pp(:,2)-pm(:,2))/(2*h)-P(:,3,k)) < 1e-8);
    da = atan2(sin(ap(:,1)-am(:,1)),cos(ap(:,1)-am(:,1)))/(2*h);
    assert(norm(da-A(:,2,k)) < 1e-8);
    assert(norm((ap(:,2)-am(:,2))/(2*h)-A(:,3,k)) < 1e-8);
end
% Nominal equality and actual Euler closed-loop acceleration.
x([1,3,5])=eta; x([2,4,6])=omega;
x([7,9,11])=[0.2;4;0.4]; x([8,10,12])=[0.1;-0.2;0.3];
[F,M] = load_lifting_payload_control_backstepping(x,A(:,:,1),P(:,:,1),Rd(:,:,1),cfg);
[Fc,Mc] = load_lifting_payload_control_rbf(x,A(:,:,1),P(:,:,1),Rd(:,:,1),zeros(30,1),dt,[],false,cfg,[],[]);
assert(norm(F-Fc)+norm(M-Mc) < 1e-12);
dx = load_lifting_payload_dynamics(x,F,M,0,false);
e = atan2(sin(eta-A(:,1,1)),cos(eta-A(:,1,1)));
ev = edot-A(:,2,1); bs=cfg.payload_control.backstepping;
expected = A(:,3,1)-bs.c1*ev-e-bs.c2*(ev+bs.c1*e);
assert(norm(Tdot*omega+T*dx([2,4,6])-expected) < 1e-10);
% True mass/inertia changes must not leak into either controller.
m_L=1.2*m_L; J_L=1.2*J_L; Jx_L=J_L(1,1); Jy_L=J_L(2,2); Jz_L=J_L(3,3);
[F2,M2] = load_lifting_payload_control_backstepping(x,A(:,:,1),P(:,:,1),Rd(:,:,1),cfg);
[F3,M3] = load_lifting_payload_control_rbf(x,A(:,:,1),P(:,:,1),Rd(:,:,1),zeros(30,1),dt,[],false,cfg,[],[]);
assert(norm(F-F2)+norm(M-M2)+norm(F-F3)+norm(M-M3) < 1e-12);
% UAV kinematics and nominal attitude acceleration use the same convention.
uav_cfg = cfg;
uav_cfg.uav_control.uav_M_backstepping.c1 = eye(3);
uav_cfg.uav_control.uav_M_backstepping.c2 = 2*eye(3);
Mu = load_lifting_UAV_control_backstepping_M(eta,omega,zeros(3,1),zeros(3,1),zeros(3,1),uav_cfg,1);
du = load_lifting_UAV_dynamics(x,zeros(3,1),Mu,0,[],1);
assert(norm(du([1,3,5])-edot) < 1e-12);
assert(norm(Tdot*omega+T*du([2,4,6])-(-edot-eta-2*(edot+eta))) < 1e-10);
% Multiple attitudes/rates and nonzero weights: switches remove all compensation.
rng(42);
for trial=1:25
    xx=randn(12,1); xx([1,3,5])=0.3*randn(3,1);
    [ff,mm]=load_lifting_payload_control_backstepping(xx,A(:,:,1000),P(:,:,1000),Rd(:,:,1000),cfg);
    [fc,mc]=load_lifting_payload_control_rbf(xx,A(:,:,1000),P(:,:,1000),Rd(:,:,1000),randn(30,1),dt,[],false,cfg,randn(18,1),randn(6,1));
    assert(norm(ff-fc)+norm(mm-mc) < 1e-12);
end
fprintf('Section 2 algebraic/trajectory checks PASSED.\n');
test_main_timing();
end

function test_main_timing()
% Short actual main-loop run, no plots/disk outputs, baseline only.
batch_output_dir = ''; %#ok<NASGU>
controller_type_override = 'backstepping'; %#ok<NASGU>
with_disturbance_override = false; %#ok<NASGU>
save_run_data = false; enable_default_plots = false; %#ok<NASGU>
simulation_Tf_override = 0.004; simulation_dt_override = 0.001; %#ok<NASGU>
load_lifting_main;
r=run_result;
assert(numel(r.time.t)==5 && r.meta.integration_intervals==4);
assert(norm(r.states.X_L(1,:)' - [0;0;0;0;0;0;0;0;4.2;0;0.1;0]) < 1e-12);
p = r.states.X_L(:,[7,9,11]);
pd = squeeze(r.desired.DesPosL_main(:,1,:))';
assert(norm(p-pd-r.save_data.Errors_Save.e_p_L,'fro') < 1e-12);
assert(norm(r.states.X_L(:,[1,3,5])-r.save_data.Errors_Save.Att_L_actual,'fro') < 1e-12);
assert(size(r.save_data.Mplant_Save,1)==5);
x=r.states.X_L(end-1,:)'; F=r.save_data.Fin_Save(end-1,:)'; M=r.save_data.Min_Save(end-1,:)';
h=r.meta.dt; tc=r.time.t(end-1);
f=@(xx,tt) load_lifting_payload_dynamics(xx,F,M,tt,false);
k1=f(x,tc); k2=f(x+h*k1/2,tc+h/2); k3=f(x+h*k2/2,tc+h/2); k4=f(x+h*k3,tc+h);
assert(norm(r.states.X_L(end,:)'-x-h*(k1+2*k2+2*k3+k4)/6) < 1e-12);
% Actual mismatch entry point and longer baseline integration smoke test.
simulation_Tf_override = 0.1;
controller_param_override = struct('experiment',struct('payload_plant',struct('mass_scale',1.2,'inertia_scale',1.2)));
load_lifting_main;
assert(all(isfinite(run_result.states.X_L(:))) && all(isfinite(run_result.states.X_UAV(:))));
assert(abs(run_result.payload_plant.mass/run_result.controller_config.payload_model.mass-1.2)<1e-12);
assert(run_result.meta.integration_intervals==100);
fprintf('Section 2 actual main-loop timing/mismatch checks PASSED.\n');
end
