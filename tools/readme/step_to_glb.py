"""STEP assembly (names, colours, positions) to GLB through the OCCT XCAF document.

    python tools/readme/step_to_glb.py <in.step> <out.glb>
"""
import sys
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString, TCollection_AsciiString
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.RWGltf import RWGltf_CafWriter
from OCP.OCP.collections import IndexedDataMap_TCollection_AsciiString_TCollection_AsciiString as Info
from OCP.Message import Message_ProgressRange
from OCP.IFSelect import IFSelect_RetDone

src, dst = sys.argv[1], sys.argv[2]
doc = TDocStd_Document(TCollection_ExtendedString("doc"))
r = STEPCAFControl_Reader()
r.SetColorMode(True); r.SetNameMode(True)
assert r.ReadFile(src) == IFSelect_RetDone
r.Transfer(doc)
BRepMesh_IncrementalMesh(r.Reader().OneShape(), 0.2, False, 0.3, True)   # mm units: 0.2 mm chord, shared with the doc shapes
w = RWGltf_CafWriter(TCollection_AsciiString(dst), True)
w.ChangeCoordinateSystemConverter().SetInputLengthUnit(0.001)
w.Perform(doc, Info(), Message_ProgressRange())
print("ok")
