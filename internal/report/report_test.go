package report

import (
	"encoding/json"
	"os"
	"path/filepath"
	"regexp"
	"strings"
	"testing"
)

// TestWriteStandalone: the written file must exist, embed a parseable JSON
// payload, reference no external/network assets and contain the playback
// controls.
func TestWriteStandalone(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "report.html")

	d := &Data{
		Title:      "test",
		NLinks:     2,
		Lengths:    []float64{1, 1},
		Limits:     [][2]float64{{-3, 3}, {-3, 3}},
		ReachInner: 0,
		ReachOuter: 2,
		Frames: []Frame{
			{
				Index: 0, Target: [2]float64{1.4, 0.2},
				Joints:   []float64{0.5, -0.4},
				Points:   [][2]float64{{0, 0}, {0.88, 0.48}, {1.4, 0.2}},
				ActualEE: [2]float64{1.4, 0.2}, ActualPhi: 0.1,
				Status: "ok", Branch: "elbow_up", MinSigma: 0.8,
			},
			{
				Index: 1, Target: [2]float64{2.3, 0},
				Joints:   []float64{0.5, -0.4},
				Points:   [][2]float64{{0, 0}, {0.88, 0.48}, {1.4, 0.2}},
				ActualEE: [2]float64{1.4, 0.2}, ActualPhi: 0.1,
				Status: "unreachable",
			},
		},
	}
	if err := Write(path, d); err != nil {
		t.Fatalf("Write: %v", err)
	}
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	html := string(raw)

	m := regexp.MustCompile(`(?s)<script id="payload" type="application/json">(.*?)</script>`).FindStringSubmatch(html)
	if m == nil {
		t.Fatal("embedded payload <script> not found")
	}
	var back Data
	if err := json.Unmarshal([]byte(m[1]), &back); err != nil {
		t.Fatalf("payload not valid JSON: %v", err)
	}
	if len(back.Frames) != 2 || back.Frames[0].Status != "ok" {
		t.Fatalf("payload round-trip mismatch: %+v", back.Frames)
	}
	if strings.Contains(html, "http://") || strings.Contains(html, "https://") ||
		regexp.MustCompile(`<script[^>]+src=`).MatchString(html) ||
		regexp.MustCompile(`<link[^>]+href=`).MatchString(html) {
		t.Error("report references external assets; it must be fully self-contained")
	}
	for _, need := range []string{"canvas", "id=\"play\"", "id=\"scrub\"", "singular", "limit_unreachable"} {
		if !strings.Contains(html, need) {
			t.Errorf("report missing required element/marker %q", need)
		}
	}
}
