from fastapi import APIRouter, HTTPException, Depends, status, Response, Request
from pydantic import BaseModel
from typing import Optional
from core.security import create_access_token, decode_access_token, verify_password, get_password_hash

router = APIRouter(prefix="/api/admin/auth", tags=["Admin Auth"])

# Hardcoded Admin User (For MVP)
# In production, this should be in DB.
ADMIN_USER = "admin"
ADMIN_PASS_HASH = get_password_hash("nan2024") # Default password

class LoginRequest(BaseModel):
    username: str
    password: str

class User(BaseModel):
    username: str

def get_current_user(request: Request):
    token = request.cookies.get("access_token")
    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
        )
    
    payload = decode_access_token(token)
    if not payload:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
        )
    
    username = payload.get("sub")
    if username != ADMIN_USER:
         raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Unknown user",
        )
    
    return User(username=username)

@router.post("/login")
def login(creds: LoginRequest, response: Response):
    if creds.username == ADMIN_USER and verify_password(creds.password, ADMIN_PASS_HASH):
        access_token = create_access_token(data={"sub": creds.username})
        
        # Set HTTP-Only Cookie
        response.set_cookie(
            key="access_token",
            value=access_token,
            httponly=True,
            max_age=86400, # 1 day
            expires=86400,
            samesite="lax",
            secure=False # Set to True in HTTPS
        )
        return {"message": "Login successful", "username": ADMIN_USER}
    else:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
        )

@router.post("/logout")
def logout(response: Response):
    response.delete_cookie("access_token")
    return {"message": "Logged out successfully"}

@router.get("/me")
def check_auth(user: User = Depends(get_current_user)):
    return {"username": user.username, "status": "authenticated"}
