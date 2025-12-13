package handlers

import (
	"io"
	"net/http"
	"net/http/httputil"
	"net/url"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
)

// ProxyChatQuery handles chat queries with potential gRPC optimization
func ProxyChatQuery(backendAddr string) gin.HandlerFunc {
	return func(c *gin.Context) {
		// For now, use HTTP proxy. Later can switch to gRPC for better performance
		proxyToHTTP(c, backendAddr, "/api/chat/query")
	}
}

// ProxyToBackend creates a reverse proxy to Python FastAPI backend
func ProxyToBackend(backendAddr string) gin.HandlerFunc {
	return func(c *gin.Context) {
		path := c.Param("path")
		fullPath := strings.TrimPrefix(c.Request.URL.Path, c.FullPath())
		if path != "" {
			fullPath = c.Request.URL.Path
		}
		proxyToHTTP(c, backendAddr, fullPath)
	}
}

func proxyToHTTP(c *gin.Context, backendAddr, path string) {
	target, err := url.Parse("http://" + backendAddr)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Invalid backend address"})
		return
	}

	proxy := httputil.NewSingleHostReverseProxy(target)
	
	// Configure timeouts
	proxy.Transport = &http.Transport{
		ResponseHeaderTimeout: 60 * time.Second,
	}

	// Modify request
	originalDirector := proxy.Director
	proxy.Director = func(req *http.Request) {
		originalDirector(req)
		req.URL.Path = path
		req.URL.RawQuery = c.Request.URL.RawQuery
		req.Host = target.Host
		
		// Forward original headers
		req.Header.Set("X-Forwarded-For", c.ClientIP())
		req.Header.Set("X-Real-IP", c.ClientIP())
		req.Header.Set("X-Forwarded-Proto", c.Request.URL.Scheme)
	}

	// Handle errors
	proxy.ErrorHandler = func(w http.ResponseWriter, req *http.Request, err error) {
		c.JSON(http.StatusBadGateway, gin.H{
			"error":   "Backend unavailable",
			"details": err.Error(),
		})
	}

	proxy.ServeHTTP(c.Writer, c.Request)
}

// AudioTranscribe handles audio transcription via C++ engine
func AudioTranscribe(cppEngineAddr string) gin.HandlerFunc {
	return func(c *gin.Context) {
		// Read audio data
		audioData, err := io.ReadAll(c.Request.Body)
		if err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": "Failed to read audio data"})
			return
		}

		// TODO: Send to C++ gRPC service
		// For now, return placeholder
		c.JSON(http.StatusOK, gin.H{
			"text":       "[Transcription placeholder - C++ engine not connected]",
			"confidence": 0.0,
			"audio_size": len(audioData),
		})
	}
}
