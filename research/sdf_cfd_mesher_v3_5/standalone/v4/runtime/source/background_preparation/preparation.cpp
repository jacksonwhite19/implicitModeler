// C17 execution of existing bounded selection and integer partition only.
// No floating arithmetic, geometry, JSON, hash, proof or acceptance cache.
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static unsigned long long selected_rows=0,partition_calls=0;
static int primitive(PyObject *value) {
    return PyLong_CheckExact(value) || PyUnicode_CheckExact(value) ||
        value==Py_True || value==Py_False || value==Py_None;
}
static PyObject *entity_row(PyObject *unused,PyObject *value) {
    PyObject *pending[256]; Py_ssize_t count=1,remaining=256;
    (void)unused; pending[0]=value;
    while (count) {
        PyObject *item=pending[--count];--remaining;
        if (remaining<0) Py_RETURN_FALSE;
        if (PyDict_CheckExact(item)) {
            Py_ssize_t at=0;PyObject *key,*child;
            // Even if all queued children were leaves, this many cannot pass
            // the original budget. This also bounds the local stack.
            if (PyDict_GET_SIZE(item)>remaining-count) Py_RETURN_FALSE;
            while (PyDict_Next(item,&at,&key,&child)) {
                if (!PyUnicode_CheckExact(key)) Py_RETURN_FALSE;
                pending[count++]=child;
            }
        } else if (PyList_CheckExact(item) || PyTuple_CheckExact(item)) {
            const int list=PyList_CheckExact(item);
            const Py_ssize_t n=list?PyList_GET_SIZE(item):PyTuple_GET_SIZE(item);
            Py_ssize_t i;
            if (n>remaining-count) Py_RETURN_FALSE;
            for (i=0;i<n;++i) pending[count++]=list?PyList_GET_ITEM(item,i):PyTuple_GET_ITEM(item,i);
        } else if (!(primitive(item) || PyFloat_CheckExact(item))) Py_RETURN_FALSE;
    }
    ++selected_rows;Py_RETURN_TRUE;
}
typedef struct { uint32_t value;int64_t first; } Signature;
static int compare(const void *a,const void *b) {
    const uint32_t x=((const Signature *)a)->value,y=((const Signature *)b)->value;
    return (x>y)-(x<y);
}
static PyObject *partition(PyObject *unused,PyObject *args) {
    PyObject *input,*output,*unique=NULL,*positions=NULL,*answer=NULL;
    Py_buffer in={0},out={0};uint32_t *table=NULL,*snapshot=NULL;Signature *first=NULL;
    const uint32_t *values=NULL;int immutable;
    Py_ssize_t count=0,capacity=0,i,n;
    (void)unused;
    if (!PyArg_ParseTuple(args,"OO",&input,&output)) return NULL;
    if (PyObject_GetBuffer(input,&in,PyBUF_CONTIG_RO|PyBUF_FORMAT)<0) return NULL;
    if (PyObject_GetBuffer(output,&out,PyBUF_CONTIG|PyBUF_FORMAT)<0) {PyBuffer_Release(&in);return NULL;}
    immutable=PyBytes_CheckExact(input);
    if (in.ndim!=1 || !in.format ||
        (immutable ? (in.itemsize!=1 || strcmp(in.format,"B") || !in.readonly) :
         (in.itemsize!=4 || (strcmp(in.format,"I") && strcmp(in.format,"L")))) ||
        out.ndim!=1 || out.itemsize!=8 || !out.format ||
        (strcmp(out.format,"q") && strcmp(out.format,"l")) ||
        in.len%4 || out.len%8 || out.len/8!=in.len/4 ||
        (uintptr_t)in.buf%4 || (uintptr_t)out.buf%8) {
        PyErr_SetString(PyExc_ValueError,"InvalidSignatureBuffers");goto done;
    }
    n=in.len/4;
    // Private-output custody is also enforced at the native API boundary.
    if (n && ((uintptr_t)in.buf<(uintptr_t)out.buf+(uintptr_t)out.len &&
        (uintptr_t)out.buf<(uintptr_t)in.buf+(uintptr_t)in.len)) {
        PyErr_SetString(PyExc_ValueError,"OverlappingSignatureBuffers");goto done;
    }
    // A held buffer is not immutable. Snapshot mutable raw callers while the
    // GIL is held; an exact bytes input is already a privately captured value.
    // Released loops never index the table through externally writable input.
    values=(const uint32_t *)in.buf;
    if (n && !immutable) {
        snapshot=(uint32_t *)PyMem_RawMalloc((size_t)in.len);
        if (!snapshot) {PyErr_NoMemory();goto done;}
        memcpy(snapshot,in.buf,(size_t)in.len);values=snapshot;
    }
    for (Py_ssize_t start=0;start<n;start+=1048576) {
        const Py_ssize_t end=start+1048576<n?start+1048576:n;
        int invalid=0;
        Py_BEGIN_ALLOW_THREADS
        for (i=start;i<end;++i) if (values[i]>=16777216u) {invalid=1;break;}
        Py_END_ALLOW_THREADS
        if (invalid) {PyErr_SetString(PyExc_ValueError,"UnsupportedSignatureDomain");goto done;}
        if (PyErr_CheckSignals()<0) goto done;
    }
    if (n) {
        table=(uint32_t *)PyMem_RawMalloc(16777216u*sizeof(uint32_t));
        if (!table) {PyErr_NoMemory();goto done;}
        memset(table,255,16777216u*sizeof(uint32_t));
        for (Py_ssize_t start=0;start<n;start+=1048576) {
          const Py_ssize_t end=start+1048576<n?start+1048576:n;
          int failed=0;
          Py_BEGIN_ALLOW_THREADS
          for (i=start;i<end;++i) {
            const uint32_t value=values[i];
            if (table[value]!=UINT32_MAX) continue;
            if (count==capacity) {
                const Py_ssize_t next=capacity?capacity*2:64;
                Signature *grown=(Signature *)PyMem_RawRealloc(first,(size_t)next*sizeof(Signature));
                if (!grown) {failed=1;break;}
                first=grown;capacity=next;
            }
            table[value]=0;first[count].value=value;first[count].first=(int64_t)i;++count;
          }
          Py_END_ALLOW_THREADS
          if (failed) {PyErr_NoMemory();goto done;}
          if (PyErr_CheckSignals()<0) goto done;
        }
        Py_BEGIN_ALLOW_THREADS
        qsort(first,(size_t)count,sizeof(Signature),compare);
        Py_END_ALLOW_THREADS
        if (PyErr_CheckSignals()<0) goto done;
    }
    unique=PyBytes_FromStringAndSize(NULL,count*4);
    if (!unique) goto done;
    positions=PyBytes_FromStringAndSize(NULL,count*8);
    if (!positions) goto done;
    for (i=0;i<count;++i) {
        memcpy(PyBytes_AS_STRING(unique)+i*4,&first[i].value,4);
        memcpy(PyBytes_AS_STRING(positions)+i*8,&first[i].first,8);
        table[first[i].value]=(uint32_t)i;
    }
    for (Py_ssize_t start=0;start<n;start+=1048576) {
        const Py_ssize_t end=start+1048576<n?start+1048576:n;
        Py_BEGIN_ALLOW_THREADS
        for (i=start;i<end;++i) ((int64_t *)out.buf)[i]=table[values[i]];
        Py_END_ALLOW_THREADS
        if (PyErr_CheckSignals()<0) goto done;
    }
    answer=PyTuple_Pack(2,unique,positions);
    if (answer) ++partition_calls;
done:
    Py_XDECREF(unique);Py_XDECREF(positions);
    PyMem_RawFree(table);PyMem_RawFree(first);PyMem_RawFree(snapshot);
    PyBuffer_Release(&out);PyBuffer_Release(&in);
    return answer;
}
static PyObject *counters(PyObject *unused,PyObject *args) {
    (void)unused;(void)args;
    return Py_BuildValue("{sK,sK}","native_rows",selected_rows,"partition_calls",partition_calls);
}
static PyMethodDef methods[]={
    {"entity_row",entity_row,METH_O,"Equivalent bounded native selection."},
    {"partition",partition,METH_VARARGS,"Equivalent integer signature partition."},
    {"counters",counters,METH_NOARGS,"Diagnostic selection only."},
    {NULL,NULL,0,NULL}
};
static PyModuleDef module={PyModuleDef_HEAD_INIT,"_v4_background_preparation",NULL,-1,methods};
PyMODINIT_FUNC PyInit__v4_background_preparation(void) {return PyModule_Create(&module);}
