#include <stdio.h>
#include <string.h>
#include <vip_lite.h>
static vip_network net = NULL;
static vip_buffer in_buf = NULL, out_buf[4];
static int n_out = 0;
static vip_status_e qry(int in, int i, vip_enum prop, void *v) {
    return in ? vip_query_input(net, i, prop, v) : vip_query_output(net, i, prop, v);
}
static int make_buf(int in, int i, vip_buffer *b) {
    vip_buffer_create_params_t p; memset(&p, 0, sizeof(p));
    if (qry(in,i,VIP_BUFFER_PROP_NUM_OF_DIMENSION,&p.num_of_dims) ||
        qry(in,i,VIP_BUFFER_PROP_SIZES_OF_DIMENSION,p.sizes) ||
        qry(in,i,VIP_BUFFER_PROP_DATA_FORMAT,&p.data_format) ||
        qry(in,i,VIP_BUFFER_PROP_QUANT_FORMAT,&p.quant_format)) return -1;
    if (p.quant_format == 1)        /* dynamic fixed point */
        qry(in,i,VIP_BUFFER_PROP_FIXED_POINT_POS,&p.quant_data.dfp.fixed_point_pos);
    else if (p.quant_format == 2) { /* asymmetric affine */
        qry(in,i,VIP_BUFFER_PROP_TF_SCALE,&p.quant_data.affine.scale);
        qry(in,i,VIP_BUFFER_PROP_TF_ZERO_POINT,&p.quant_data.affine.zeroPoint);
    }
    return vip_create_buffer(&p, sizeof(p), b) == VIP_SUCCESS ? 0 : -2;
}
int npu_init(const char *nb) {
    vip_uint32_t cnt = 0;
    if (vip_init() != VIP_SUCCESS) return -1;
    if (vip_create_network(nb, 0, VIP_CREATE_NETWORK_FROM_FILE, &net) != VIP_SUCCESS) return -2;
    vip_query_network(net, VIP_NETWORK_PROP_OUTPUT_COUNT, &cnt); n_out = (int)cnt;
    if (make_buf(1, 0, &in_buf)) return -3;
    for (int i = 0; i < n_out; i++) if (make_buf(0, i, &out_buf[i])) return -4;
    if (vip_prepare_network(net) != VIP_SUCCESS) return -5;
    if (vip_set_input(net, 0, in_buf) != VIP_SUCCESS) return -6;
    for (int i = 0; i < n_out; i++) if (vip_set_output(net, i, out_buf[i]) != VIP_SUCCESS) return -7;
    return n_out;
}
int   npu_in_size(void)    { return (int)vip_get_buffer_size(in_buf); }
int   npu_out_size(int i)  { return (int)vip_get_buffer_size(out_buf[i]); }
float npu_out_scale(int i) { float s = 0; vip_query_output(net, i, VIP_BUFFER_PROP_TF_SCALE, &s); return s; }
int   npu_out_zp(int i)    { int z = 0; vip_query_output(net, i, VIP_BUFFER_PROP_TF_ZERO_POINT, &z); return z; }
int npu_run(const unsigned char *in, unsigned char *o0, unsigned char *o1) {
    unsigned char *outs[2] = {o0, o1};
    void *p = vip_map_buffer(in_buf);
    memcpy(p, in, vip_get_buffer_size(in_buf));
    vip_flush_buffer(in_buf, VIP_BUFFER_OPER_TYPE_FLUSH);
    vip_unmap_buffer(in_buf);
    if (vip_run_network(net) != VIP_SUCCESS) return -1;
    for (int i = 0; i < n_out && i < 2; i++) {
        vip_flush_buffer(out_buf[i], VIP_BUFFER_OPER_TYPE_INVALIDATE);
        p = vip_map_buffer(out_buf[i]);
        memcpy(outs[i], p, vip_get_buffer_size(out_buf[i]));
        vip_unmap_buffer(out_buf[i]);
    }
    return 0;
}
void npu_close(void) {
    vip_finish_network(net);
    vip_destroy_buffer(in_buf);
    for (int i = 0; i < n_out; i++) vip_destroy_buffer(out_buf[i]);
    vip_destroy_network(net);
    vip_destroy();
}
