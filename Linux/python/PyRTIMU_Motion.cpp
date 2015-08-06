////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech
//  Copyright (c) 2014, avishorp
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// RTIUMULib Python Module - RTHumidity type implementation
///////////////////////////////////////////////////////

#include "PyMotion.h"
#include "PyRTIMU.h"

// Forwards
///////////
struct RTIMU_Motion;
static void RTIMU_Motion_dealloc(RTIMU_Motion* self);
static PyObject* RTIMU_Motion_new(PyTypeObject *type, PyObject *args, PyObject *kwds);
static int RTIMU_Motion_init(RTIMU_Motion *self, PyObject *args, PyObject *kwds);

static PyObject* RTIMU_Motion_lmn(RTIMU_Motion* self);
static PyObject* RTIMU_Motion_abc(RTIMU_Motion* self);
static PyObject* RTIMU_Motion_xyz(RTIMU_Motion* self, PyObject *args, PyObject *keywds);

// The RTIMU_Motion struct
////////////////////////////
static PyMethodDef RTIMU_Motion_methods[] = {
    {"save", (PyCFunction)RTIMU_Motion_lmn, METH_NOARGS, "Save settings to a file"},
    {"load", (PyCFunction)RTIMU_Motion_abc, METH_NOARGS, "Load settings from a file"},
    {"getMotionData", (PyCFunction)RTIMU_Motion_xyz, METH_VARARGS|METH_KEYWORDS,
    "Try do discover and auto-set a connected IMU"},
    { NULL }
};

// The RTIMU_Settings struct
////////////////////////////
static PyMethodDef RTIMU_Motion_methods[] = {

    //////// motionInit
    {"motionInit", (PyCFunction)([] (PyObject *self, PyObject* args) -> PyObject* {
        if (((RTIMU_Motion*)self)->val == NULL)
            return PyBool_FromLong(0);
        else
            return PyBool_FromLong(((RTIMU_Motion*)self)->val->motionInit());
        }),
        METH_NOARGS,
    "Set up the motion module" },

    //////// motionReset

    //////// detectMotion
    {"detectMotion", (PyCFunction)([] (PyObject *self, PyObject* args) -> PyObject* {
        return PyBool_FromLong(((RTIMU_Motion*)self)->val->detectMotion());
        }),
    METH_NOARGS,
    "Return true if sensor is moving" },
    
    //////// getMotionData
    {"getMotionData", (PyCFunction)([] (PyObject *self, PyObject* args) -> PyObject* {
        const MOTION_DATA& mdata = ((RTIMU_Motion*)self)->val->getMotionData();
        return Py_BuildValue("{s:(d,d,d), s:(d,d,d), s:(d,d,d), s:O}",
                 "WorldAcceleration", mdata.worldAcceleration.x(), mdata.worldAcceleration.y(), mdata.worldAcceleration.z(),
                 "WorldVelocity", mdata.worldVelocity.x(), mdata.worldVelocity.y(), mdata.worldVelocity.z(),
                 "WorldPosition", mdata.worldPosition.x(), mdata.worldPosition.y(), mdata.worldPosition.z(),
                 "motion", PyBool_FromLong(mdata.motion));
        }),
    METH_NOARGS,
    "Return motion data" },
	

    //////// updateVelocityPosition
    {"motionyUpdate", (PyCFunction)([] (PyObject *self, PyObject* args) -> PyObject* {
        return PyBool_FromLong(((RTIMU_Motion*)self)->val->updateVelocityPosition());
        }),
    METH_NOARGS,
    "Update Motion Engine" },

    { NULL }
};


static PyTypeObject RTIMU_Motion_type = {
#if PY_MAJOR_VERSION >= 3
    PyVarObject_HEAD_INIT(NULL, 0)
#else
    PyObject_HEAD_INIT(NULL)
    0,                          /*ob_size*/
#endif
     "RTIMU.Motion",            /*tp_name*/
    sizeof(RTIMU_Motion),       /*tp_basicsize*/
    0,                          /*tp_itemsize*/
    (destructor)RTIMU_Motion_dealloc,  /*tp_dealloc*/
    0,                          /*tp_print*/
    0,                          /*tp_getattr*/
    0,                          /*tp_setattr*/
    0,                          /*tp_compare*/
    0,                          /*tp_repr*/
    0,                          /*tp_as_number*/
    0,                          /*tp_as_sequence*/
    0,                          /*tp_as_mapping*/
    0,                          /*tp_hash */
    0,                          /*tp_call*/
    0,                          /*tp_str*/
    0,                          /*tp_getattro*/
    0,                          /*tp_setattro*/
    0,                          /*tp_as_buffer*/
    Py_TPFLAGS_DEFAULT,         /*tp_flags*/
    "RTIMU.Motion object",      /* tp_doc */
    0,                          /* tp_traverse */
    0,                          /* tp_clear */
    0,                          /* tp_richcompare */
    0,                          /* tp_weaklistoffset */
    0,                          /* tp_iter */
    0,                          /* tp_iternext */
    RTIMU_Motion_methods,       /* tp_methods */
    0,                          /* tp_members */
    0,                          /* tp_getset */
    0,                          /* tp_base */
    0,                          /* tp_dict */
    0,                          /* tp_descr_get */
    0,                          /* tp_descr_set */
    0,                          /* tp_dictoffset */
    (initproc)RTIMU_Motion_init, /* tp_init */
    0,                          /* tp_alloc */
    RTIMU_Motion_new,       /* tp_new */
};

static void RTIMU_Motion_dealloc(RTIMU_RTHumidity* self)
{
    if (self->val != NULL)
        delete self->val;
}

static PyObject* RTIMU_Motion_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
    RTIMU_Motion* self;

    // Allocate the object memory
    self = (RTIMU_Motion*)type->tp_alloc(type, 0);

    if (self != NULL) {
        // Set the value field to NULL. The Motion Settings object
        // creation is deferred to init
        self->val = NULL;
    }

    return (PyObject*)self;
}

static int RTIMU_Motion_init(RTIMU_RTHumidity *self, PyObject *args, PyObject *kwds)
{
    PyObject* settings;

    // The user should pass settings pointer as an argument
    if (!PyArg_ParseTuple(args, "O", &settings))
        return -1;

    // Create an RTHumidity object
    self->val = new RTMotion(((RTIMU_Settings*)settings)->val);

    return 0;
}

int RTIMU_Motion_create(PyObject* module)
{
    // Prepare the type structure
    if (PyType_Ready(&RTIMU_Motion_type) < 0)
        return -1;

    // Insert it into the module
    Py_INCREF(&RTIMU_Motion_type);
    PyModule_AddObject(module, "Motion", (PyObject*)&RTIMU_Motion_type);
    return 0;
}

bool RTIMU_Motion_typecheck(PyObject* obj)
{
    return PyObject_TypeCheck(obj, (PyTypeObject*)&RTIMU_Motion_type);
}