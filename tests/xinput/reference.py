"""Independent fixture generator; needs OpenSSL only when regenerating.

No production code is imported or called. DES/3DES use OpenSSL; SHA-1 uses
hashlib. Packet framing, key derivation, salted MAC and ACR are implemented
here. Public protocol constants and PARVE/ACR definitions are from
InvoxiPlayGames/libxsm3 (LGPL-2.1-or-later), copyright 2013 oct0xor and
2022-2023 InvoxiPlayGames; see OpenPuck/src/libxsm3/LICENSE.txt.

Prints JSON to stdout; the checked-in fixture is consumed without OpenSSL.
These are synthetic retail-key transcripts, not captures from hardware.
"""

import hashlib
import json
import subprocess


KEY_D = bytes.fromhex("e35bfb1ccdad325bf70e07fd623da7c4")
KEY_E = bytes.fromhex("8f2908380b5bfe687c26462a51f2bc19")
ROOT_23 = bytes.fromhex("828078683a523a9810f40c127066dcba")
ROOT_24 = bytes.fromhex("66621a78f8609c8a269a04aed85c1ec8")
SERIAL = b"012345ABCDEF"
RANDOM = bytes(range(0x90, 0xA0))
SBOX = bytes.fromhex(
    "b03d9b70f3c78060739f6cc0f13dbb40b3c83714df49dad4482278806ecde700"
    "818668e15d7c542c557bef48427b3b68e3dbaac00fa99620950593949af6a364"
    "5dcc7600e50819e88d29d74c219117f4bc6ab38083c6d4909bae0efe2e4af200"
    "7388d94066c5d40857b18948dc54fc436a2687b8095fce80e40b059c24f3dee2"
    "3eec388aa255a4504e4be9587f9f7d80230c4d80054426b8e9d8bce6763a6ea4"
    "19dec2d0c4bcc35c59df16463970f4ee2d585aa817866b6029584dd25f287ad8"
    "8e79ea8294333181d922d510da92a07d3ddaac1ca25331b83c965200826b56a0"
    "d3c240c71b7fdc017270b18c01090936fc97eadee30dae7ee30dae7e33698040"
)
PLAINTEXT = bytes.fromhex(
    "d1d2f2806eba0cc0b6c4c9d861751d1a3f9558bed80de2c0d0217920652d9940"
    "3c9652001b7fdc01821c13d833698040fc97eade08ea14dceb0f6a186f782cb0"
    "d3c240c7826b56a0190936e07270b18ce30dae7e50a52be2c9afc7701c298056"
    "24f066fa022b58988fe4d13c6e382affb8fa35b05249c5b466fa47556c8d4008"
)


def crypt(key, data, encrypt=True):
    # DES parity bits do not affect OpenSSL's key schedule.
    args = ["openssl", "enc", "-des-ede-cbc", "-nopad", "-nosalt",
            "-K", key[:16].hex(), "-iv", "0000000000000000"]
    if not encrypt:
        args.append("-d")
    return subprocess.run(args, input=data, capture_output=True,
                          check=True).stdout


def xor(a, b):
    return bytes(x ^ y for x, y in zip(a, b))


def mac(key, data, salt=None):
    block = bytes(8)
    des_key = key[:8] * 2
    if salt is not None:
        counter = (int.from_bytes(salt[:8], "big") + 1) % (1 << 64)
        salt[:8] = counter.to_bytes(8, "big")
        block = crypt(des_key, salt[:8])
    for offset in range(0, len(data), 8):
        block = crypt(des_key, xor(block, data[offset:offset + 8]))
    return crypt(key, xor(block, b"\x80" + bytes(7)))


def parve(key, data):
    state = list(data) + [data[0]]
    for round_number in range(8, 0, -1):
        for index in range(8):
            value = SBOX[(key[index] + state[index] + round_number) & 255]
            value = (value + state[index + 1]) & 255
            state[index + 1] = ((value << 1) | (value >> 7)) & 255
        state[0] = state[8]
    return bytes(state[:8])


def acr(console, identity, key):
    cd = parve(key, identity[:4] + console[:4])
    ab = parve(key, identity[16:24])
    for offset in range(0, 128, 8):
        ab = parve(key, xor(ab, PLAINTEXT[offset:offset + 8]))
    modulus = 0x7FFFFFFF
    a, b = (int.from_bytes(ab[i:i + 4], "big") % modulus for i in (0, 4))
    c, d = (int.from_bytes(cd[i:i + 4], "big") % modulus for i in (0, 4))
    accumulator = total = 0
    for offset in range(0, 128, 8):
        left = int.from_bytes(PLAINTEXT[offset:offset + 4], "big")
        right = int.from_bytes(PLAINTEXT[offset + 4:offset + 8], "big")
        accumulator = ((accumulator + left * 0xE79A9C1) * a + b) % modulus
        total += accumulator
        accumulator = ((right + accumulator) * c + d) % modulus
        total += accumulator
    result = ((accumulator + b) % modulus).to_bytes(4, "big")
    result += ((total + d) % modulus).to_bytes(4, "big")
    return xor(result, ab)


def packet(payload, magic=b"IL"):
    checksum = 0
    for value in payload:
        checksum ^= value
    return magic + b"\0\0" + bytes([len(payload)]) + payload + bytes([checksum])


def transcript(number):
    console = bytes(range(0x10 + number * 0x20, 0x18 + number * 0x20))
    nonce = bytes(range(number * 0x20, number * 0x20 + 16))
    identification = packet(SERIAL + bytes.fromhex("0080025e048e0203000101"), b"IK")
    identity = bytearray(32)
    identity[:15] = identification[5:20]
    identity[16:21] = identification[20:25]
    identity[21] = identification[27]
    identity[22:24] = identification[25:27]
    digest = hashlib.sha1(console).digest()
    key1 = crypt(ROOT_23, digest[:16])
    key2 = crypt(ROOT_24, digest[4:20])
    session1 = crypt(key1, nonce)
    session2 = crypt(key2, nonce[8:] + nonce[:8])
    encrypted = crypt(KEY_D, nonce + console)
    init = packet(encrypted + mac(KEY_E, encrypted)[4:])
    encrypted = crypt(session1, RANDOM + nonce)
    init_response = packet(encrypted + acr(console, identity,
                                           mac(session2, encrypted)))

    # The console recovers the controller nonce and hash from the reply.
    recovered = crypt(session1, init_response[5:37], False)
    assert recovered[16:] == nonce
    controller = recovered[:16]
    init_hash = hashlib.sha1(recovered).digest()
    salt = bytearray(controller[12:] + nonce[12:] + nonce[8:])
    verifies = []
    replies = []
    for iteration in range(3):
        fresh = bytes(range(0x60 + iteration * 8, 0x68 + iteration * 8))
        salt[8:] = fresh
        encrypted = crypt(controller, fresh)
        verifies.append(packet(encrypted + mac(init_hash, encrypted, salt)))
        encrypted = crypt(session1, acr(console, identity, fresh))
        replies.append(packet(encrypted + mac(session2, encrypted, salt)))
    return {"console": console.hex(), "nonce": nonce.hex(),
            "identification": identification.hex(), "init": init.hex(),
            "init_response": init_response.hex(),
            "verify": [v.hex() for v in verifies],
            "verify_response": [v.hex() for v in replies]}


def main():
    # FIPS DES example, exercised through two-key EDE with identical keys.
    key = bytes.fromhex("133457799bbcdff1") * 2
    assert crypt(key, bytes.fromhex("0123456789abcdef")).hex() == (
        "85e813540f0ab405")
    key = bytes(range(16))
    salt = bytearray.fromhex("ffffffffffffffff1020304050607080")
    salted_mac = mac(key, bytes(range(24)), salt)
    fixture = {"consoles": [transcript(0), transcript(1)],
               "mac_wrap": salted_mac.hex(), "salt_wrap": salt.hex()}
    print(json.dumps(fixture, indent=2))


if __name__ == "__main__":
    main()