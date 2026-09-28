package rrtstar

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestWriteHTMLReport(t *testing.T) {
	res, err := Run(demoOptions(42, 800))
	if err != nil {
		t.Fatal(err)
	}
	dir := t.TempDir()
	path := filepath.Join(dir, "report.html")
	if err := WriteHTMLReport(path, res); err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	html := string(raw)

	// Must be a single self-contained file: no external resource references.
	for _, banned := range []string{"http://", "https://", "src=\"//", "<link", "cdn."} {
		if strings.Contains(html, banned) {
			t.Fatalf("report contains forbidden external reference %q", banned)
		}
	}
	if !strings.Contains(html, "<!DOCTYPE html>") ||
		!strings.Contains(html, "application/json") ||
		!strings.Contains(html, "gotoFrame") {
		t.Fatal("report missing expected structural markers")
	}
	// The JSON payload must not be able to close its script tag early.
	if strings.Contains(html, `</script>"`) {
		t.Fatal("payload appears re-encoded as a JS string (template context bug)")
	}
}
