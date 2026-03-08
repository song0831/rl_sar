// Stub for libcusparseLt.so.0
// libcusparseLt is not available on Jetson/aarch64 but PyTorch 2.5 links against it.
// All functions return 0 (CUSPARSE_STATUS_SUCCESS) so the library loads successfully.
// Actual sparse matrix operations via cusparseLt will silently no-op.

int cusparseLtInit(void *h) { return 0; }
int cusparseLtDenseDescriptorInit(void *h, void *d, int r, int c, int ld, int ab, int ct, int o) { return 0; }
int cusparseLtStructuredDescriptorInit(void *h, void *d, int r, int c, int ld, int ab, int ct, int o, int sp) { return 0; }
int cusparseLtMatDescriptorDestroy(void *d) { return 0; }
int cusparseLtMatmulDescriptorInit(void *h, void *d, int op, void *a, void *b, void *c, void *dd, int algo) { return 0; }
int cusparseLtMatmulDescSetAttribute(void *h, void *d, int attr, const void *data, unsigned long sz) { return 0; }
int cusparseLtMatmulAlgSelectionInit(void *h, void *algSel, void *d, int algo) { return 0; }
int cusparseLtMatmulAlgGetAttribute(void *h, void *algSel, int attr, void *data, unsigned long sz) { return 0; }
int cusparseLtMatmulAlgSetAttribute(void *h, void *algSel, int attr, const void *data, unsigned long sz) { return 0; }
int cusparseLtMatmulGetWorkspace(void *h, void *plan, unsigned long *ws) { if (ws) *ws = 0; return 0; }
int cusparseLtMatmulPlanInit(void *h, void *plan, void *d, void *algSel, unsigned long ws) { return 0; }
int cusparseLtMatmulPlanDestroy(void *plan) { return 0; }
int cusparseLtMatmul(void *h, void *plan, const void *alpha, const void *a, const void *b,
                     const void *beta, const void *c, void *dd, void **ws, void **streams, int ns) { return 0; }
int cusparseLtMatmulSearch(void *h, void *plan, const void *alpha, const void *a, const void *b,
                            const void *beta, const void *c, void *dd, void **ws, void **streams, int ns) { return 0; }
int cusparseLtSpMMACompress2(void *h, void *sd, int isSA, int op,
                              const void *in, void *out, void *col, void *s) { return 0; }
int cusparseLtSpMMACompressedSize2(void *h, void *sd, unsigned long *ccs, unsigned long *cs) {
    if (ccs) *ccs = 0;
    if (cs)  *cs  = 0;
    return 0;
}
