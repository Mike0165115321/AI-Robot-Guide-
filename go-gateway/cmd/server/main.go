package main

import (
	"log"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"
	"context"

	"go-gateway/internal/handlers"
	"go-gateway/internal/middleware"

	"github.com/gin-gonic/gin"
)

const (
	defaultPort        = ":8080"
	pythonBackendAddr  = "localhost:9090"
	cppAudioEngineAddr = "localhost:50051"
)

func main() {
	// Set Gin to release mode in production
	if os.Getenv("GIN_MODE") == "release" {
		gin.SetMode(gin.ReleaseMode)
	}

	router := gin.Default()

	// Apply global middleware
	router.Use(middleware.CORS())
	router.Use(middleware.RateLimiter(100)) // 100 requests per second

	// Health check
	router.GET("/health", func(c *gin.Context) {
		c.JSON(http.StatusOK, gin.H{
			"status":  "healthy",
			"service": "go-gateway",
			"version": "1.0.0",
			"time":    time.Now().Format(time.RFC3339),
		})
	})

	// API routes - proxy to Python backend
	api := router.Group("/api")
	{
		// Chat endpoints
		api.POST("/chat/query", handlers.ProxyChatQuery(pythonBackendAddr))
		
		// Admin endpoints
		api.Any("/admin/*path", handlers.ProxyToBackend(pythonBackendAddr))
		
		// Avatar endpoints
		api.Any("/avatar/*path", handlers.ProxyToBackend(pythonBackendAddr))
	}

	// WebSocket endpoint for real-time communication
	router.GET("/ws/avatar", handlers.WebSocketHandler())

	// Serve frontend static files
	router.Static("/assets", "../frontend/assets")
	router.StaticFile("/", "../frontend/index.html")
	router.StaticFile("/chat", "../frontend/chat.html")
	router.StaticFile("/admin", "../frontend/admin.html")
	router.StaticFile("/robot_avatar", "../frontend/robot_avatar.html")

	// Start server
	srv := &http.Server{
		Addr:         defaultPort,
		Handler:      router,
		ReadTimeout:  30 * time.Second,
		WriteTimeout: 30 * time.Second,
	}

	// Graceful shutdown
	go func() {
		log.Printf("🚀 Go Gateway starting on %s", defaultPort)
		log.Printf("📡 Python backend: %s", pythonBackendAddr)
		log.Printf("🔊 C++ Audio Engine: %s", cppAudioEngineAddr)
		if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("❌ Server failed: %v", err)
		}
	}()

	// Wait for interrupt signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit
	log.Println("⏳ Shutting down server...")

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	if err := srv.Shutdown(ctx); err != nil {
		log.Fatalf("❌ Server forced to shutdown: %v", err)
	}

	log.Println("✅ Server exited gracefully")
}
