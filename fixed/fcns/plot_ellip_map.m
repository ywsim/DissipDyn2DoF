function h = plot_ellip_map(M, origin, scale, varargin)

if isrow(origin)
    origin = origin';
end


sz_pts = 200;
q_ = linspace(0, 2*pi, sz_pts);
u_ = [cos(q_);sin(q_)];

ellip_ = scale*M*u_;
ellip_x = ellip_(1,:) + origin(1);
ellip_y = ellip_(2,:) + origin(2);
h = plot(ellip_x, ellip_y, varargin{:});
end