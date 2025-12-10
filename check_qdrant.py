# check_qdrant.py
import sys
import os

# Add the backend path to the sys.path
backend_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'backend'))
if backend_path not in sys.path:
    sys.path.insert(0, backend_path)

from app.services.qdrant_service import get_collection_info

if __name__ == "__main__":
    info = get_collection_info()
    print(info)
