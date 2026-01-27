from fastapi import FastAPI

from .endpoints import router
from .ros_lifecycle import register_lifecycle_handlers

app = FastAPI()
register_lifecycle_handlers(app)
app.include_router(router)
