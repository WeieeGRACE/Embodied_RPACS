"""Generate the OLED font ROM image (768 x 8: 128 chars, 6 columns each).

The glyph table mirrors the case table that used to live in
system_oled_text.v; column k of a glyph is byte k of the 48-bit constant.
All 768 entries are emitted (unmapped characters read as 00) so simulation
never sees X values from $readmemh.
"""
GLYPHS = {
    "0": 0x3E4549513E00, "1": 0x00407F420000, "2": 0x465149614200,
    "3": 0x314B45412100, "4": 0x107F12141800, "5": 0x394545452700,
    "6": 0x3049494A3C00, "7": 0x030509710100, "8": 0x364949493600,
    "9": 0x1E2949490600, "A": 0x7C1211127C00, "B": 0x364949497F00,
    "C": 0x224141413E00, "D": 0x1C2241417F00, "E": 0x414949497F00,
    "F": 0x010909097F00, "S": 0x324949492600, "m": 0x780418047C00,
    "G": 0x7A4949413E00, "H": 0x7F0808087F00, "I": 0x00417F410000,
    "L": 0x404040407F00, "M": 0x7F020C027F00, "N": 0x7F1008047F00,
    "O": 0x3E4141413E00, "P": 0x060909097F00, "R": 0x462919097F00,
    "T": 0x01017F010100, "U": 0x3F4040403F00, "W": 0x3F4038403F00,
    "J": 0x013F41412000, "K": 0x412214087F00, "Q": 0x5E2151413E00,
    "V": 0x1F2040201F00, "X": 0x631408146300, "Z": 0x434549516100,
    ">": 0x00081C224100, "/": 0x030408106000, "=": 0x141414141400,
    "Y": 0x070870080700, ":": 0x000036360000, "+": 0x08083E080800,
    "-": 0x080808080800, ".": 0x000060600000,
}

rom = [0] * 768
for ch, value in GLYPHS.items():
    base = ord(ch) * 6
    for col in range(6):
        byte = (value >> (col * 8)) & 0x7F
        assert byte == (value >> (col * 8)) & 0xFF, f"{ch} col {col} sets bit 7"
        rom[base + col] = byte

with open("src/ui/oled_font.hex", "w", encoding="ascii") as f:
    for byte in rom:
        f.write(f"{byte:02x}\n")
print(f"wrote src/ui/oled_font.hex with {len(rom)} entries, {len(GLYPHS)} glyphs")
