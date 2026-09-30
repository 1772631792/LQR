"""Step 3 validation oracle ONLY. Never used as a fallback for a missing DLL."""
import numpy as np
from simulation.design_data import load_design


class PythonReferenceController:
    def __init__(self):
        data = load_design()
        self.k = np.asarray(data['K'][0])
        self.limit = data['params']['output_limit']

    def update(self, state, reference):
        return float(np.clip(-self.k @ (state-reference), -self.limit, self.limit))
