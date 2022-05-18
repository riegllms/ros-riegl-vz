# Copyright 2008 Willow Garage, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import array
from collections import namedtuple
import sys
from typing import Iterable, List, NamedTuple, Optional

import numpy as np
try:
    from numpy.lib.recfunctions import (structured_to_unstructured, unstructured_to_structured)
except ImportError:
    from sensor_msgs_py.numpy_compat import (structured_to_unstructured,
                                             unstructured_to_structured)

from riegl_vz_interfaces.msg import VoxelGrid, VoxelField
from std_msgs.msg import Header


_DATATYPES = {}
_DATATYPES[VoxelField.UINT8] = np.dtype(np.uint8)
_DATATYPES[VoxelField.UINT32] = np.dtype(np.uint32)
_DATATYPES[VoxelField.UINT64] = np.dtype(np.uint64)
_DATATYPES[VoxelField.FLOAT32] = np.dtype(np.float32)
_DATATYPES[VoxelField.FLOAT64] = np.dtype(np.float64)

DUMMY_FIELD_PREFIX = 'unnamed_field'


def read_voxels(
        voxelgrid: VoxelGrid,
        field_names: Optional[List[str]] = None,
        skip_nans: bool = False,
        uvs: Optional[Iterable] = None,
        reshape_organized_voxelgrid: bool = False) -> np.ndarray:
    """
    Read voxels from a riegl_vz_interfaces.VoxelGrid message.
    :param voxelgrid: The voxels to read from riegl_vz_interfaces.VoxelGrid.
    :param field_names: The names of fields to read. If None, read all fields.
                        (Type: Iterable, Default: None)
    :param skip_nans: If True, then don't return any voxel with a NaN value.
                      (Type: Bool, Default: False)
    :param uvs: If specified, then only return the voxels at the given
        coordinates. (Type: Iterable, Default: None)
    :param reshape_organized_voxelgrid: Returns the array as an 2D organized voxel voxelgrid if set.
    :return: Structured NumPy array containing all voxels.
    """
    assert isinstance(voxelgrid, VoxelGrid), \
        'Cloud is not a riegl_vz_interfaces.msg.VoxelGrid'

    # Cast bytes to numpy array
    voxels = np.ndarray(
        shape=(voxelgrid.width * voxelgrid.height, ),
        dtype=dtype_from_fields(voxelgrid.fields),
        buffer=voxelgrid.data)

    # Keep only the requested fields
    if field_names is not None:
        assert all(field_name in voxels.dtype.names for field_name in field_names), \
            'Requests field is not in the fields of the VoxelGrid!'
        # Mask fields
        voxels = voxels[list(field_names)]

    # Swap array if byte order does not match
    if bool(sys.byteorder != 'little') != bool(voxelgrid.is_bigendian):
        voxels = voxels.byteswap(inplace=True)

    # Check if we want to drop voxels with nan values
    if skip_nans and not voxelgrid.is_dense:
        # Init mask which selects all voxels
        not_nan_mask = np.ones(len(voxels), dtype=bool)
        for field_name in voxels.dtype.names:
            # Only keep voxels without any non values in the mask
            not_nan_mask = np.logical_and(
                not_nan_mask, ~np.isnan(voxels[field_name]))
        # Select these voxels
        voxels = voxels[not_nan_mask]

    # Select voxels indexed by the uvs field
    if uvs is not None:
        # Don't convert to numpy array if it is already one
        if not isinstance(uvs, np.ndarray):
            uvs = np.fromiter(uvs, int)
        # Index requested voxels
        voxels = voxels[uvs]

    # Cast into 2d array if voxelgrid is 'organized'
    if reshape_organized_voxelgrid and voxelgrid.height > 1:
        voxels = voxels.reshape(voxelgrid.width, voxelgrid.height)

    return voxels


def read_voxels_numpy(
        voxelgrid: VoxelGrid,
        field_names: Optional[List[str]] = None,
        skip_nans: bool = False,
        uvs: Optional[Iterable] = None,
        reshape_organized_voxelgrid: bool = False) -> np.ndarray:
    """
    Read equally typed fields from riegl_vz_interfaces.VoxelGrid message as a unstructured numpy array.
    This method is better suited if one wants to perform math operations
    on e.g. all x,y,z fields.
    But it is limited to fields with the same dtype as unstructured numpy arrays
    only contain one dtype.
    :param voxelgrid: The voxels to read from riegl_vz_interfaces.VoxelGrid.
    :param field_names: The names of fields to read. If None, read all fields.
                        (Type: Iterable, Default: None)
    :param skip_nans: If True, then don't return any voxel with a NaN value.
                      (Type: Bool, Default: False)
    :param uvs: If specified, then only return the voxels at the given
        coordinates. (Type: Iterable, Default: None)
    :param reshape_organized_voxelgrid: Returns the array as an 2D organized voxels if set.
    :return: Numpy array containing all voxels.
    """
    assert all(voxelgrid.fields[0].datatype == field.datatype for field in voxelgrid.fields[1:]), \
        'All fields need to have the same datatype. Use `read_voxels()` otherwise.'
    structured_numpy_array = read_voxels(
        voxelgrid, field_names, skip_nans, uvs, reshape_organized_voxelgrid)
    return structured_to_unstructured(structured_numpy_array)


def read_voxels_list(
        voxelgrid: VoxelGrid,
        field_names: Optional[List[str]] = None,
        skip_nans: bool = False,
        uvs: Optional[Iterable] = None) -> List[NamedTuple]:
    """
    Read voxels from a riegl_vz_interfaces.VoxelGrid message.
    This function returns a list of namedtuples. It operates on top of the
    read_voxels method. For more efficient access use read_voxels directly.
    :param voxelgrid: The voxels to read from. (Type: riegl_vz_interfaces.VoxelGrid)
    :param field_names: The names of fields to read. If None, read all fields.
                        (Type: Iterable, Default: None)
    :param skip_nans: If True, then don't return any voxel with a NaN value.
                      (Type: Bool, Default: False)
    :param uvs: If specified, then only return the voxels at the given
                coordinates. (Type: Iterable, Default: None]
    :return: List of namedtuples containing the values for each voxel
    """
    assert isinstance(voxelgrid, VoxelGrid), \
        'voxelgrid is not a riegl_vz_interfaces.msg.VoxelGrid'

    if field_names is None:
        field_names = [f.name for f in voxelgrid.fields]

    Point = namedtuple('Point', field_names)

    return [Point._make(p) for p in read_voxels(voxelgrid, field_names,
                                                skip_nans, uvs)]


def dtype_from_fields(fields: Iterable[VoxelField]) -> np.dtype:
    """
    Convert a Iterable of riegl_vz_interfaces.msg.VoxelField messages to a np.dtype.
    :param fields: The voxels fields.
                   (Type: iterable of riegl_vz_interfaces.msg.VoxelField)
    :returns: NumPy datatype
    """
    # Create a lists containing the names, offsets and datatypes of all fields
    field_names = []
    field_offsets = []
    field_datatypes = []
    for i, field in enumerate(fields):
        # Datatype as numpy datatype
        datatype = _DATATYPES[field.datatype]
        # Name field
        if field.name == '':
            name = f'{DUMMY_FIELD_PREFIX}_{i}'
        else:
            name = field.name
        # Handle fields with count > 1 by creating subfields with a suffix consiting
        # of "_" followed by the subfield counter [0 -> (count - 1)]
        assert field.count > 0, "Can't process fields with count = 0."
        for a in range(field.count):
            # Add suffix if we have multiple subfields
            if field.count > 1:
                subfield_name = f'{name}_{a}'
            else:
                subfield_name = name
            assert subfield_name not in field_names, 'Duplicate field names are not allowed!'
            field_names.append(subfield_name)
            # Create new offset that includes subfields
            field_offsets.append(field.offset + a * datatype.itemsize)
            field_datatypes.append(datatype.str)

    # Create a tuple for each field containing name and data type
    return np.dtype({
        'names': field_names,
        'formats': field_datatypes,
        'offsets': field_offsets,
    })


def create_voxelgrid(
        header: Header,
        fields: Iterable[VoxelField],
        voxels: Iterable) -> VoxelGrid:
    """
    Create a riegl_vz_interfaces.msg.VoxelGrid message.
    :param header: The voxels header. (Type: std_msgs.msg.Header)
    :param fields: The voxels fields.
                   (Type: iterable of riegl_vz_interfaces.msg.VoxelField)
    :param voxels: The voxels voxels. List of iterables, i.e. one iterable
                   for each voxel, with the elements of each iterable being the
                   values of the fields for that voxel (in the same order as
                   the fields parameter)
    :return: The voxels as riegl_vz_interfaces.msg.VoxelGrid
    """
    # Check if input is numpy array
    if isinstance(voxels, np.ndarray):
        # Check if this is an unstructured array
        if voxels.dtype.names is None:
            assert all(fields[0].datatype == field.datatype for field in fields[1:]), \
                'All fields need to have the same datatype. Pass a structured NumPy array \
                    with multiple dtypes otherwise.'
            # Convert unstructured to structured array
            voxels = unstructured_to_structured(
                voxels,
                dtype=dtype_from_fields(fields))
        else:
            assert voxels.dtype == dtype_from_fields(fields), \
                'VoxelFields and structured NumPy array dtype do not match for all fields! \
                    Check their field order, names and types.'
    else:
        # Cast python objects to structured NumPy array (slow)
        voxels = np.array(
            # Points need to be tuples in the structured array
            list(map(tuple, voxels)),
            dtype=dtype_from_fields(fields))

    # Handle organized voxelgrids
    assert len(voxels.shape) <= 2, \
        'Too many dimensions for organized voxelgrid! \
            Points can only be organized in max. two dimensional space'
    height = 1
    width = voxels.shape[0]
    # Check if input voxels are an organized voxelgrid (2D array of voxels)
    if len(voxels.shape) == 2:
        height = voxels.shape[1]

    # Convert numpy voxels to array.array
    memory_view = memoryview(voxels)
    casted = memory_view.cast('B')
    array_array = array.array('B')
    array_array.frombytes(casted)

    # Put everything together
    voxelgrid = VoxelGrid(
        header=header,
        height=height,
        width=width,
        is_dense=False,
        is_bigendian=sys.byteorder != 'little',
        fields=fields,
        voxel_step=voxels.dtype.itemsize,
        row_step=(voxels.dtype.itemsize * width))
    # Set voxelgrid via property instead of the constructor because of the bug described in
    # https://github.com/ros2/common_interfaces/issues/176
    voxelgrid.data = array_array
    return voxelgrid


def create_voxelgrid_xyz32(header: Header, voxels: Iterable) -> VoxelGrid:
    """
    Create a riegl_vz_interfaces.msg.VoxelGrid message with (x, y, z) fields.
    :param header: The voxels header. (Type: std_msgs.msg.Header)
    :param voxels: The voxels voxels. (Type: Iterable)
    :return: The voxels as riegl_vz_interfaces.msg.VoxelGrid.
    """
    fields = [VoxelField(name='x', offset=0,
                         datatype=VoxelField.FLOAT64, count=1),
              VoxelField(name='y', offset=8,
                         datatype=VoxelField.FLOAT64, count=1),
              VoxelField(name='z', offset=16,
                         datatype=VoxelField.FLOAT64, count=1)]
    return create_voxelgrid(header, fields, voxels)
